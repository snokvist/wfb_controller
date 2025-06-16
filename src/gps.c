/*
 * gps2mavlink.c  —  tiny GPS-to-MAVLink bridge
 *
 *   gcc -O2 -Wall -o gps2mavlink gps2mavlink.c -lm
 *
 *   ./gps2mavlink                 # /dev/ttyS2 → 127.0.0.1:14550
 *   ./gps2mavlink -d /dev/ttyUSB0 -v       # human-readable
 *   ./gps2mavlink -d /dev/ttyUSB0 -v -v    # + raw NMEA
 */

#define _POSIX_C_SOURCE 200809L

/* ------------ system ---------------------------------------------------- */
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

/* ------------ minimal MAVLink-v1 helper (GLOBAL_POSITION_INT) ----------- */
#define MAV_MAGIC          0xFE
#define MAV_MSG_ID_GPI     33
#define MAV_GPI_LEN        28
#define MAV_GPI_EXTRA_CRC  104

static uint8_t mav_seq = 0;

static inline void crc_acc(uint8_t d, uint16_t *crc)
{
    d ^= *crc & 0xFF;
    d ^= d << 4;
    *crc = (*crc >> 8) ^ ((uint16_t)d << 8) ^ ((uint16_t)d << 3) ^ ((uint16_t)d >> 4);
}

static uint16_t mav_crc(uint8_t *hdr, const uint8_t *pl, uint8_t len, uint8_t extra)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < 5; ++i) crc_acc(hdr[i], &c);
    for (int i = 0; i < len; ++i) crc_acc(pl[i], &c);
    crc_acc(extra, &c);
    return c;
}

static size_t mav_pack_gpi(uint8_t *buf,
                           uint32_t time_ms,
                           int32_t  lat, int32_t lon,
                           int32_t  alt_mm, int32_t rel_alt_mm,
                           int16_t  vx, int16_t vy, int16_t vz,
                           uint16_t hdg)
{
    uint8_t *p = buf;
    *p++ = MAV_MAGIC;
    *p++ = MAV_GPI_LEN;
    *p++ = mav_seq++;
    *p++ = 1;                       /* sysid  */
    *p++ = 1;                       /* compid */
    *p++ = MAV_MSG_ID_GPI;

#define PUTU32(v)  do{uint32_t _=(v); memcpy(p,&_,4); p+=4;}while(0)
#define PUTS32     PUTU32
#define PUTS16(v)  do{int16_t _=(v);  memcpy(p,&_,2); p+=2;}while(0)
#define PUTU16(v)  PUTS16(v)

    PUTU32(time_ms);
    PUTS32(lat);
    PUTS32(lon);
    PUTS32(alt_mm);
    PUTS32(rel_alt_mm);
    PUTS16(vx);
    PUTS16(vy);
    PUTS16(vz);
    PUTU16(hdg);

#undef PUTU32
#undef PUTS32
#undef PUTS16
#undef PUTU16

    uint8_t hdr[5] = { MAV_GPI_LEN, (uint8_t)(mav_seq-1), 1, 1, MAV_MSG_ID_GPI };
    uint16_t crc = mav_crc(hdr, buf + 6, MAV_GPI_LEN, MAV_GPI_EXTRA_CRC);
    memcpy(p, &crc, 2);
    return 6 + MAV_GPI_LEN + 2;
}

/* ------------ helpers --------------------------------------------------- */
static double degmin_to_deg(double dm)
{
    int d = (int)(dm / 100);
    double m = dm - d * 100;
    return d + m / 60.0;
}

static void cfg_serial(int fd, speed_t baud)
{
    struct termios t;
    tcgetattr(fd, &t);
    cfsetispeed(&t, baud);
    cfsetospeed(&t, baud);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | CSTOPB);
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_lflag = 0;
    t.c_oflag = 0;
    tcsetattr(fd, TCSANOW, &t);
}

/* ------------------------------------------------------------------------ */
static uint32_t millis(void)          /* monotonic since program start */
{
    static struct timespec t0 = {0,0};
    struct timespec now;
    if (!t0.tv_sec) clock_gettime(CLOCK_MONOTONIC, &t0);
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec  - t0.tv_sec )*1000U +
           (now.tv_nsec - t0.tv_nsec)/1000000U;
}

/* ------------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    const char *dev = "/dev/ttyS2";
    char host[64]   = "127.0.0.1";
    int  port       = 14550;
    int  verbose    = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:u:vh")) != -1) {
        switch (opt) {
        case 'd': dev = optarg; break;
        case 'u':
            if (sscanf(optarg,"%63[^:]:%d",host,&port)!=2){
                fprintf(stderr,"Bad -u HOST:PORT\n"); return 1;}
            break;
        case 'v': verbose++; break;
        default:
            fprintf(stderr,"Usage: %s [-d /dev/ttyUSB0] [-u host:port] [-v [-v]]\n", argv[0]);
            return 0;
        }
    }

    /* serial */
    int fd = open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(dev); return 1; }
    cfg_serial(fd, B115200);

    /* UDP */
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket"); return 1; }
    struct sockaddr_in dst = { .sin_family = AF_INET,
                               .sin_port   = htons(port) };
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
        fprintf(stderr,"bad IP %s\n",host); return 1;
    }

    fprintf(stderr,"Serial %s  →  UDP %s:%d\n", dev, host, port);

    /* runtime vars ------------------------------------------------------- */
    char buf[1024]={0}; int len=0;

    double lat=0, lon=0, alt=0, spd_kn=0, crs_deg=-1;
    int    valid_rmc=0, fix_quality=0, sats=0;

    for (;;) {
        /* --- read with 100 ms timeout ----------------------------------- */
        fd_set rf; struct timeval tv={0,100000};
        FD_ZERO(&rf); FD_SET(fd,&rf);
        if (select(fd+1,&rf,NULL,NULL,&tv)<0 && errno!=EINTR){perror("select");break;}

        int updated = 0;                /* set when we parse at least one line */

        if (FD_ISSET(fd,&rf)) {
            ssize_t n = read(fd,buf+len,sizeof(buf)-1-len);
            if (n>0){ len+=n; buf[len]=0; }

            char *eol,*p=buf;
            while((eol=strchr(p,'\n'))) {
                *eol=0;

                if (verbose>1) fprintf(stderr,"NMEA: %s\n",p);

                /* --------- RMC --------- */
                if (!strncmp(p,"$GPRMC",6)) {
                    char *t[13]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<13; q=strtok(NULL,",")) t[i++]=q;
                    if (i>=9) {
                        valid_rmc = (t[2][0]=='A');
                        if (valid_rmc) {
                            double rl = atof(t[3]), rlo = atof(t[5]);
                            lat = degmin_to_deg(rl);
                            lon = degmin_to_deg(rlo);
                            if (t[4][0]=='S') lat=-lat;
                            if (t[6][0]=='W') lon=-lon;
                        }
                        spd_kn  = atof(t[7]);
                        crs_deg = atof(t[8]);
                        updated = 1;
                    }
                }
                /* --------- GGA --------- */
                else if (!strncmp(p,"$GPGGA",6)) {
                    char *t[15]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<15; q=strtok(NULL,",")) t[i++]=q;
                    if (i>=10) {
                        fix_quality = atoi(t[6]);
                        sats        = atoi(t[7]);
                        alt         = atof(t[9]);
                        updated = 1;
                    }
                }
                p=eol+1;
            }
            memmove(buf,p,len-(p-buf));
            len -= (p-buf);
        }

        /* emit only when we parsed a fresh sentence --------------------- */
        if (!updated) continue;

        int good = (valid_rmc || fix_quality>0);
        if (!good) continue;

        /* build & send --------------------------------------------------- */
        uint32_t tms = millis();
        double spd_ms  = spd_kn*0.514444;
        double crs_rad = crs_deg*M_PI/180.0;
        int32_t lat_e7 = (int32_t)(lat*1e7);
        int32_t lon_e7 = (int32_t)(lon*1e7);
        int32_t alt_mm = (int32_t)(alt*1000.0);
        int16_t vx = (int16_t)(spd_ms*cos(crs_rad)*100.0);
        int16_t vy = (int16_t)(spd_ms*sin(crs_rad)*100.0);

        uint16_t hdg = (crs_deg>=0.0)? (uint16_t)(crs_deg*100) : UINT16_MAX;

        uint8_t frame[36];
        size_t  sz = mav_pack_gpi(frame,tms,
                                  lat_e7,lon_e7,
                                  alt_mm,alt_mm,
                                  vx,vy,0,hdg);
        sendto(s,frame,sz,0,(struct sockaddr*)&dst,sizeof(dst));

        if (verbose) {
            printf("t=%u ms  fix=%s  lat=%.7f  lon=%.7f  alt=%.2f m  "
                   "spd=%.2f kn  crs=%.1f°  sats=%d\n",
                   tms, valid_rmc?"A":(fix_quality?"GGA":"void"),
                   lat, lon, alt, spd_kn, crs_deg, sats);
            fflush(stdout);
        }

        valid_rmc = 0;      /* wait for next RMC */
        /* keep fix_quality so GGA-only solutions still work */
    }
    return 0;
}
