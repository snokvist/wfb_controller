/*
 * gps2mavlink.c  –  very small GPS-to-MAVLink bridge
 *
 *  ▸ No external mavlink headers         ▸ No fancy filtering
 *  ▸ Builds a MAVLink-v1 frame “by hand” ▸ Verbose console output
 *
 *  gcc -O2 -Wall -o gps2mavlink gps2mavlink.c -lm
 *
 *  ./gps2mavlink             # /dev/ttyS2  → 127.0.0.1:14550
 *  ./gps2mavlink -d /dev/ttyUSB0 -u 192.168.10.2:14551 -v
 */

#define _POSIX_C_SOURCE 200809L
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

/* --------------------------------------------------------------------------
   Simple MAVLink-v1 helper (GLOBAL_POSITION_INT only, msg id = 33, len = 28)
   -------------------------------------------------------------------------- */
#define MAV_MAGIC          0xFE
#define MAV_GPS_MSG_ID     33
#define MAV_GPS_EXTRA_CRC  104         /* extra CRC for GLOBAL_POSITION_INT */

static uint8_t mav_seq = 0;            /* packet sequence number */

/* CRC-X25 (polynomial 0x1021) ------------------------------------------------*/
static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    data ^= (uint8_t)(*crc & 0x00FF);
    data ^= (data << 4);
    *crc = (*crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4);
}

static uint16_t mav_crc(uint8_t len_hdr_fields[5], const uint8_t *payload,
                        uint8_t payload_len, uint8_t extra)
{
    uint16_t crc = 0xFFFF;
    /* header except magic */
    for (int i = 0; i < 5; ++i) crc_accumulate(len_hdr_fields[i], &crc);
    /* payload */
    for (int i = 0; i < payload_len; ++i) crc_accumulate(payload[i], &crc);
    /* extra byte per message id */
    crc_accumulate(extra, &crc);
    return crc;
}

/* Packs message into buf; returns frame length (always 6+28+2 = 36) */
static size_t mav_pack_global_position_int(uint8_t *buf,
                                           uint32_t time_boot_ms,
                                           int32_t  lat, int32_t lon,
                                           int32_t  alt_mm, int32_t rel_alt_mm,
                                           int16_t  vx,  int16_t vy,
                                           int16_t  vz,  uint16_t hdg)
{
    uint8_t *p = buf;
    /* ---- header ---- */
    *p++ = MAV_MAGIC;                 /* 0 */
    *p++ = 28;                        /* payload length               */
    *p++ = mav_seq++;                 /* packet sequence              */
    *p++ = 1;                         /* sysid (change if you like)   */
    *p++ = 1;                         /* compid                       */
    *p++ = MAV_GPS_MSG_ID;            /* message id                   */

    /* ---- payload ---- : little-endian */
#define PUT_U32(v)  do{ uint32_t _v=(v); memcpy(p,&_v,4); p+=4; }while(0)
#define PUT_S32     PUT_U32
#define PUT_S16(v)  do{ int16_t  _v=(v); memcpy(p,&_v,2); p+=2; }while(0)
#define PUT_U16(v)  PUT_S16(v)

    PUT_U32(time_boot_ms);
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

    /* ---- CRC ---- */
    uint8_t hdr_min[5] = { 28, (uint8_t)(mav_seq - 1), 1, 1, MAV_GPS_MSG_ID };
    uint16_t crc = mav_crc(hdr_min, buf + 6, 28, MAV_GPS_EXTRA_CRC);
    memcpy(p, &crc, 2);               /* little-endian */
    return 6 + 28 + 2;
}

/* --------------------------------------------------------------------------
   Very small NMEA helpers (just GPRMC + GPGGA, no checksum checks)
   -------------------------------------------------------------------------- */
static double nmea_degmin_to_deg(double dm)
{
    int d = (int)(dm / 100);
    double m = dm - d * 100;
    return d + m / 60.0;
}

/* -------------------------------------------------------------------------- */
static void configure_serial(int fd, speed_t baud)
{
    struct termios t;
    tcgetattr(fd, &t);
    cfsetispeed(&t, baud);
    cfsetospeed(&t, baud);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= (CLOCAL | CREAD);  t.c_cflag &= ~(PARENB | CSTOPB);
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_lflag = 0;                  /* raw */
    t.c_oflag = 0;
    tcsetattr(fd, TCSANOW, &t);
}

/* -------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *dev = "/dev/ttyS2";
    char udp_host[64] = "127.0.0.1";
    int  udp_port     = 14550;
    int  verbose      = 0;

    /* --- very tiny CLI ---------------------------------------------------- */
    int opt;
    while ((opt = getopt(argc, argv, "d:u:vh")) != -1)
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'u': if(sscanf(optarg,"%63[^:]:%d",udp_host,&udp_port)!=2){
                      fprintf(stderr,"Bad -u HOST:PORT\n"); return 1;}break;
        case 'v': verbose = 1; break;
        default:
            fprintf(stderr,
              "gps2mavlink [-d /dev/ttyUSB0] [-u host:port] [-v]\n"); return 0;
        }

    /* --- serial port ------------------------------------------------------ */
    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("serial"); return 1; }
    configure_serial(fd, B115200);

    /* --- UDP socket ------------------------------------------------------- */
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket"); return 1; }
    struct sockaddr_in dest = { .sin_family = AF_INET,
                                .sin_port   = htons(udp_port) };
    inet_pton(AF_INET, udp_host, &dest.sin_addr);

    /* --- runtime vars ----------------------------------------------------- */
    char buf[1024] = {0};  int len = 0;
    uint32_t start_ms = (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);

    /* last parsed values */
    double lat=0, lon=0, alt=0, spd_kn=0, crs_deg=0; int fix_ok=0;

    fprintf(stderr,"Serial %s  →  UDP %s:%d\n", dev, udp_host, udp_port);

    /* --- main loop -------------------------------------------------------- */
    for (;;)
    {
        fd_set rfds;  struct timeval tv = {0, 100000}; /* 100 ms */
        FD_ZERO(&rfds); FD_SET(fd,&rfds);
        int rv = select(fd+1,&rfds,NULL,NULL,&tv);
        if (rv < 0 && errno!=EINTR) { perror("select"); break; }

        if (FD_ISSET(fd, &rfds))
        {
            ssize_t n = read(fd, buf+len, sizeof(buf)-1-len);
            if (n > 0) { len += n; buf[len] = 0; }

            /* process complete lines */
            char *eol, *ptr = buf;
            while ((eol = strchr(ptr, '\n'))) {
                *eol = 0;
                if (strncmp(ptr,"$GPRMC",6)==0) {
                    /* $GPRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,sss.ss,ccc.cc,... */
                    char *tok[13]={0}; int i=0;
                    for(char *p=strtok(ptr,","); p && i<13; p=strtok(NULL,",")) tok[i++]=p;
                    if (i>=9 && tok[2][0]=='A') {
                        double raw_lat = atof(tok[3]);
                        double raw_lon = atof(tok[5]);
                        lat = nmea_degmin_to_deg(raw_lat);
                        lon = nmea_degmin_to_deg(raw_lon);
                        if (tok[4][0]=='S') lat = -lat;
                        if (tok[6][0]=='W') lon = -lon;
                        spd_kn   = atof(tok[7]);
                        crs_deg  = atof(tok[8]);
                        fix_ok   = 1;
                    }
                }
                else if (strncmp(ptr,"$GPGGA",6)==0) {
                    /* $GPGGA,...,M (altitude) */
                    char *tok[11]={0}; int i=0;
                    for(char *p=strtok(ptr,","); p && i<11; p=strtok(NULL,",")) tok[i++]=p;
                    if (i>=10) alt = atof(tok[9]);   /* metres */
                }
                ptr = eol + 1;
            }
            /* move remaining bytes to front */
            memmove(buf, ptr, len - (ptr - buf));
            len -= (ptr - buf);
        }

        /* if latest GPRMC & GPGGA seen → emit MAVLink */
        if (fix_ok) {
            uint32_t now_ms = (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC)
                              - start_ms;
            double  spd_ms  = spd_kn * 0.514444;
            double  crs_rad = crs_deg * M_PI / 180.0;

            int32_t lat_e7 = (int32_t)(lat * 1e7);
            int32_t lon_e7 = (int32_t)(lon * 1e7);
            int32_t alt_mm = (int32_t)(alt * 1000.0);

            int16_t vx = (int16_t)(spd_ms * cos(crs_rad) * 100); /* cm/s */
            int16_t vy = (int16_t)(spd_ms * sin(crs_rad) * 100);
            int16_t vz = 0;

            uint16_t hdg = (fix_ok && crs_deg>=0) ? (uint16_t)(crs_deg*100)
                                                  : UINT16_MAX;

            uint8_t frame[36];
            size_t  sz = mav_pack_global_position_int(frame,
                                 now_ms, lat_e7, lon_e7,
                                 alt_mm, alt_mm, vx, vy, vz, hdg);

            sendto(s, frame, sz, 0, (struct sockaddr*)&dest, sizeof(dest));

            /* -------- verbose human-readable dump ------------------------- */
            if (verbose) {
                printf("MAV 33: t=%ums  lat=%.7f  lon=%.7f  alt=%.2fm "
                       "spd=%.2fkn  crs=%.1f°  sats=?\n",
                       now_ms, lat, lon, alt, spd_kn, crs_deg);
                fflush(stdout);
            }
            fix_ok = 0; /* wait for next GPRMC */
        }
    }
    return 0;
}
