/*
 * gps2mavlink.c  —  tiny GPS-to-MAVLink bridge
 *
 *   gcc -O2 -Wall -o gps2mavlink gps2mavlink.c -lm
 *
 *   ./gps2mavlink                        # /dev/ttyS2 → 127.0.0.1:14550
 *   ./gps2mavlink -d /dev/ttyUSB0 -v     # verbose
 *   ./gps2mavlink -d /dev/ttyUSB0 -v -v  # + raw NMEA dump
 */

#define _POSIX_C_SOURCE 200809L

/* ---------------- system ------------------------------------------------- */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- minimal MAVLink-v1 helper (GLOBAL_POSITION_INT) -------- */
#define MAV_MAGIC          0xFE
#define MAV_MSG_ID_GPI     33
#define MAV_GPI_PAYLOAD    28
#define MAV_GPI_EXTRA_CRC  104            /* per MAVLink spec */

static uint8_t mav_seq = 0;               /* rolling packet sequence */

/* CRC-X25 (0x1021) -------------------------------------------------------- */
static inline void crc_acc(uint8_t data, uint16_t *crc)
{
    data ^= (uint8_t)(*crc & 0x00FF);
    data ^= (data << 4);
    *crc = (*crc >> 8) ^ ((uint16_t)data << 8) ^ ((uint16_t)data << 3) ^ ((uint16_t)data >> 4);
}

static uint16_t mav_crc(uint8_t hdr[5], const uint8_t *pl, uint8_t len, uint8_t extra)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 5; ++i) crc_acc(hdr[i], &crc);
    for (int i = 0; i < len; ++i) crc_acc(pl[i], &crc);
    crc_acc(extra, &crc);
    return crc;
}

/* build GLOBAL_POSITION_INT frame; returns frame size = 6+28+2 = 36 bytes */
static size_t mav_pack_gpi(uint8_t *buf,
                           uint32_t time_ms,
                           int32_t  lat, int32_t lon,
                           int32_t  alt_mm, int32_t rel_alt_mm,
                           int16_t  vx, int16_t vy, int16_t vz,
                           uint16_t hdg)
{
    uint8_t *p = buf;
    *p++ = MAV_MAGIC;
    *p++ = MAV_GPI_PAYLOAD;   /* length */
    *p++ = mav_seq++;
    *p++ = 1;                 /* sysid  */
    *p++ = 1;                 /* compid */
    *p++ = MAV_MSG_ID_GPI;

#define PUT_U32(v)  do{ uint32_t _=(v); memcpy(p,&_,4); p+=4; }while(0)
#define PUT_S32     PUT_U32
#define PUT_S16(v)  do{ int16_t _=(v); memcpy(p,&_,2); p+=2; }while(0)
#define PUT_U16(v)  PUT_S16(v)

    PUT_U32(time_ms);
    PUT_S32(lat);
    PUT_S32(lon);
    PUT_S32(alt_mm);
    PUT_S32(rel_alt_mm);
    PUT_S16(vx);
    PUT_S16(vy);
    PUT_S16(vz);
    PUT_U16(hdg);

#undef PUT_U32
#undef PUT_S32
#undef PUT_S16
#undef PUT_U16

    uint8_t hdr_min[5] = { MAV_GPI_PAYLOAD, (uint8_t)(mav_seq-1), 1, 1, MAV_MSG_ID_GPI };
    uint16_t crc = mav_crc(hdr_min, buf + 6, MAV_GPI_PAYLOAD, MAV_GPI_EXTRA_CRC);
    memcpy(p, &crc, 2);
    return 6 + MAV_GPI_PAYLOAD + 2;
}

/* ----------------- helpers ---------------------------------------------- */
static double nmea_degmin_to_deg(double dm)
{
    int d = (int)(dm / 100);
    double m = dm - d * 100;
    return d + m / 60.0;
}

static void configure_serial(int fd, speed_t baud)
{
    struct termios t;
    tcgetattr(fd, &t);
    cfsetispeed(&t, baud);
    cfsetospeed(&t, baud);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~(PARENB | CSTOPB);
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_lflag = 0;
    t.c_oflag = 0;
    tcsetattr(fd, TCSANOW, &t);
}

/* ----------------- main -------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *dev = "/dev/ttyS2";
    char udp_host[64] = "127.0.0.1";
    int  udp_port     = 14550;
    int  verbose      = 0;

    /* CLI */
    int opt;
    while ((opt = getopt(argc, argv, "d:u:vh")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'u':
            if (sscanf(optarg, "%63[^:]:%d", udp_host, &udp_port) != 2) {
                fprintf(stderr, "Bad -u HOST:PORT\n");
                return 1;
            }
            break;
        case 'v': verbose++; break;
        default:
            fprintf(stderr,
              "Usage: %s [-d /dev/ttyUSB0] [-u host:port] [-v [-v]]\n", argv[0]);
            return 0;
        }
    }

    /* serial */
    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(dev); return 1; }
    configure_serial(fd, B115200);

    /* UDP */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in dst = { .sin_family = AF_INET,
                               .sin_port = htons(udp_port) };
    if (inet_pton(AF_INET, udp_host, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad IP %s\n", udp_host); return 1;
    }

    fprintf(stderr, "Serial %s  →  UDP %s:%d\n", dev, udp_host, udp_port);

    /* runtime vars */
    uint32_t t0_ms = (uint32_t)(clock()*1000/CLOCKS_PER_SEC);
    char buf[1024] = {0}; int len = 0;

    double lat=0, lon=0, alt=0, spd_kn=0, crs_deg=-1;
    int    valid_rmc = 0, fix_quality = 0, satellites = 0;

    /* main loop */
    for (;;)
    {
        fd_set rfds; struct timeval tv={0,100000};
        FD_ZERO(&rfds); FD_SET(fd,&rfds);
        if (select(fd+1,&rfds,NULL,NULL,&tv) < 0 && errno != EINTR) {
            perror("select"); break;
        }

        /* read serial */
        if (FD_ISSET(fd,&rfds)) {
            ssize_t n = read(fd, buf+len, sizeof(buf)-1-len);
            if (n > 0) { len += n; buf[len] = 0; }

            char *eol, *ptr = buf;
            while ((eol = strchr(ptr, '\n'))) {
                *eol = 0;

                if (verbose > 1)
                    fprintf(stderr, "NMEA: %s\n", ptr);

                /* -------- GPRMC -------- */
                if (!strncmp(ptr, "$GPRMC", 6)) {
                    char *tok[13]={0}; int i=0;
                    for (char *p=strtok(ptr, ","); p && i<13; p=strtok(NULL, ",")) tok[i++]=p;
                    if (i >= 9) {
                        valid_rmc = (tok[2][0] == 'A');
                        if (valid_rmc) {
                            double raw_lat = atof(tok[3]);
                            double raw_lon = atof(tok[5]);
                            lat = nmea_degmin_to_deg(raw_lat);
                            lon = nmea_degmin_to_deg(raw_lon);
                            if (tok[4][0] == 'S') lat = -lat;
                            if (tok[6][0] == 'W') lon = -lon;
                        }
                        spd_kn  = atof(tok[7]);
                        crs_deg = atof(tok[8]);
                    }
                }
                /* -------- GPGGA -------- */
                else if (!strncmp(ptr, "$GPGGA", 6)) {
                    char *tok[15]={0}; int i=0;
                    for (char *p=strtok(ptr, ","); p && i<15; p=strtok(NULL, ",")) tok[i++]=p;
                    if (i >= 10) {
                        fix_quality = atoi(tok[6]);      /* 0 = no fix */
                        satellites  = atoi(tok[7]);
                        alt         = atof(tok[9]);      /* metres */
                    }
                }

                ptr = eol + 1;
            }
            memmove(buf, ptr, len - (ptr - buf));
            len -= (ptr - buf);
        }

        /* decide if we have something usable */
        int good_fix = (valid_rmc || fix_quality > 0);

        if (good_fix) {
            uint32_t now_ms = (uint32_t)(clock()*1000/CLOCKS_PER_SEC) - t0_ms;

            double spd_ms  = spd_kn * 0.514444;
            double crs_rad = crs_deg * M_PI / 180.0;
            int32_t lat_e7 = (int32_t)(lat * 1e7);
            int32_t lon_e7 = (int32_t)(lon * 1e7);
            int32_t alt_mm = (int32_t)(alt * 1000.0);

            int16_t vx = (int16_t)(spd_ms * cos(crs_rad) * 100.0); /* cm/s */
            int16_t vy = (int16_t)(spd_ms * sin(crs_rad) * 100.0);
            int16_t vz = 0;

            uint16_t hdg = (crs_deg >= 0.0) ? (uint16_t)(crs_deg * 100) : UINT16_MAX;

            uint8_t frame[36];
            size_t sz = mav_pack_gpi(frame, now_ms,
                                     lat_e7, lon_e7,
                                     alt_mm, alt_mm,
                                     vx, vy, vz, hdg);

            sendto(sock, frame, sz, 0, (struct sockaddr*)&dst, sizeof(dst));

            if (verbose) {
                printf("t=%u ms  fix=%s  lat=%.7f  lon=%.7f  alt=%.1f m  "
                       "spd=%.2f kn  crs=%.1f°  sats=%d\n",
                       now_ms, valid_rmc ? "A" : (fix_quality ? "GGA" : "void"),
                       lat, lon, alt, spd_kn, crs_deg, satellites);
                fflush(stdout);
            }

            /* reset only what must become fresh before next packet */
            valid_rmc = 0;
            /* leave fix_quality as-is so GGA-only streams continue until updated */
        }
    }
    return 0;
}
