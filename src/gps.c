/* gps2mavlink.c  –  tiny GPS → MAVLink-v1 bridge
 *
 *  *  GLOBAL_POSITION_INT  (msg 33)  • 5–10 Hz (only on $GPRMC)
 *  *  GPS_STATUS           (msg 25)  • 1 Hz max (after each full $GPGSV burst or $GPGSA)
 *
 *  gcc -O2 -Wall -o gps2mavlink gps2mavlink.c -lm
 *  ./gps2mavlink                  # /dev/ttyS2 → 127.0.0.1:14550
 *  ./gps2mavlink -d /dev/ttyUSB0 -v          # human output
 *  ./gps2mavlink -d /dev/ttyUSB0 -v -v       # + raw NMEA dump
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

/* ------------ MAVLink mini-helper -------------------------------------- */
#define MAV_MAGIC 0xFE

/* msg-33 */
#define MAV_ID_GPI        33
#define MAV_LEN_GPI       28
#define MAV_EXTRA_GPI     104
/* msg-25 */
#define MAV_ID_GPSSTAT    25
#define MAV_LEN_GPSSTAT   101
#define MAV_EXTRA_GPSSTAT 23

static uint8_t mav_seq = 0;

static inline void crc_acc(uint8_t d, uint16_t *crc)
{
    d ^= *crc & 0xFF;
    d ^= d << 4;
    *crc = (*crc >> 8) ^ ((uint16_t)d << 8) ^ ((uint16_t)d << 3) ^ ((uint16_t)d >> 4);
}
static uint16_t mav_crc(uint8_t *hdr, const uint8_t *pl,
                        uint8_t len, uint8_t extra)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < 5; ++i) crc_acc(hdr[i], &c);
    for (int i = 0; i < len; ++i) crc_acc(pl[i], &c);
    crc_acc(extra, &c);
    return c;
}
static uint8_t *mav_start(uint8_t *p, uint8_t len, uint8_t id)
{
    *p++ = MAV_MAGIC;
    *p++ = len;
    *p++ = mav_seq++;
    *p++ = 1;                 /* sysid  */
    *p++ = 1;                 /* compid */
    *p++ = id;
    return p;
}

#define PUTU32(v) do{uint32_t _=(v); memcpy(p,&_,4); p+=4;}while(0)
#define PUTS32     PUTU32
#define PUTS16(v) do{int16_t  _=(v); memcpy(p,&_,2); p+=2;}while(0)
#define PUTU16(v) PUTS16(v)

/* ---------- pack GLOBAL_POSITION_INT (33) ------------------------------- */
static size_t pack_gpi(uint8_t *buf,
                       uint32_t tms,
                       int32_t  lat, int32_t lon,
                       int32_t  alt_mm, int32_t rel_alt_mm,
                       int16_t  vx,  int16_t vy, int16_t vz,
                       uint16_t hdg)
{
    uint8_t *p = mav_start(buf, MAV_LEN_GPI, MAV_ID_GPI);
    PUTU32(tms);  PUTS32(lat); PUTS32(lon);
    PUTS32(alt_mm); PUTS32(rel_alt_mm);
    PUTS16(vx); PUTS16(vy); PUTS16(vz); PUTU16(hdg);
    uint8_t hdr[5] = { MAV_LEN_GPI,(uint8_t)(mav_seq-1),1,1,MAV_ID_GPI };
    uint16_t crc = mav_crc(hdr, buf+6, MAV_LEN_GPI, MAV_EXTRA_GPI);
    memcpy(p,&crc,2);
    return 6 + MAV_LEN_GPI + 2;
}

/* ---------- pack GPS_STATUS (25) --------------------------------------- */
static size_t pack_gps_status(uint8_t *buf,
                              uint8_t sats_vis,
                              uint8_t prn[20], uint8_t used[20],
                              uint8_t elev[20], uint8_t az[20],
                              uint8_t snr[20])
{
    uint8_t *p = mav_start(buf, MAV_LEN_GPSSTAT, MAV_ID_GPSSTAT);
    *p++ = sats_vis;
    memcpy(p, prn,  20); p += 20;
    memcpy(p, used, 20); p += 20;
    memcpy(p, elev, 20); p += 20;
    memcpy(p, az,   20); p += 20;
    memcpy(p, snr,  20); p += 20;
    uint8_t hdr[5] = { MAV_LEN_GPSSTAT,(uint8_t)(mav_seq-1),1,1,MAV_ID_GPSSTAT };
    uint16_t crc = mav_crc(hdr, buf+6, MAV_LEN_GPSSTAT, MAV_EXTRA_GPSSTAT);
    memcpy(p,&crc,2);
    return 6 + MAV_LEN_GPSSTAT + 2;
}

/* ------------ helpers --------------------------------------------------- */
static double dm_to_deg(double dm)          /* ddmm.mmmm -> degrees */
{
    int d = (int)(dm/100);
    return d + (dm - d*100) / 60.0;
}
static void cfg_serial(int fd, speed_t b)
{
    struct termios t; tcgetattr(fd,&t);
    cfsetispeed(&t,b); cfsetospeed(&t,b);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cflag |= CLOCAL | CREAD; t.c_cflag &= ~(PARENB|CSTOPB);
    t.c_iflag &= ~(IXON|IXOFF|IXANY);
    t.c_lflag = t.c_oflag = 0;
    tcsetattr(fd,TCSANOW,&t);
}
static uint32_t millis(void)
{
    static struct timespec t0={0,0};
    struct timespec now;
    if(!t0.tv_sec) clock_gettime(CLOCK_MONOTONIC,&t0);
    clock_gettime(CLOCK_MONOTONIC,&now);
    return (now.tv_sec  - t0.tv_sec )*1000U +
           (now.tv_nsec - t0.tv_nsec)/1000000U;
}

/* ------------ satellite table (max 20) --------------------------------- */
static uint8_t sat_prn [20]={0};
static uint8_t sat_used[20]={0};
static uint8_t sat_elev[20]={0};
static uint8_t sat_az  [20]={0};
static uint8_t sat_snr [20]={0};
static int     sats_vis_gsv = 0;

static int idx_for_prn(uint8_t prn)
{
    for(int i=0;i<20;i++) if(sat_prn[i]==prn) return i;
    for(int i=0;i<20;i++) if(sat_prn[i]==0){ sat_prn[i]=prn; return i; }
    return -1;
}

/* ----------------------------------------------------------------------- */
int main(int argc,char**argv)
{
    const char *dev="/dev/ttyS2";
    char host[64]="127.0.0.1";
    int  port=14550, verbose=0;

    int opt;
    while((opt=getopt(argc,argv,"d:u:vh"))!=-1){
        switch(opt){
        case 'd': dev=optarg; break;
        case 'u':
            if(sscanf(optarg,"%63[^:]:%d",host,&port)!=2){
                fprintf(stderr,"Bad -u HOST:PORT\n"); return 1; }
            break;
        case 'v': verbose++; break;
        default:
            fprintf(stderr,"Usage: %s [-d dev] [-u host:port] [-v [-v]]\n",argv[0]);
            return 0;
        }
    }

    int fd=open(dev,O_RDONLY|O_NOCTTY|O_NONBLOCK);
    if(fd<0){perror(dev);return 1;}
    cfg_serial(fd,B115200);

    int sock=socket(AF_INET,SOCK_DGRAM,0);
    if(sock<0){perror("socket");return 1;}
    struct sockaddr_in dst={.sin_family=AF_INET,.sin_port=htons(port)};
    if(inet_pton(AF_INET,host,&dst.sin_addr)!=1){fprintf(stderr,"Bad IP\n");return 1;}

    fprintf(stderr,"Serial %s → UDP %s:%d\n",dev,host,port);

    /* -------- runtime vars --------------------------------------------- */
    char buf[1024]={0}; int len=0;
    double lat=0,lon=0,alt=0,spd_kn=0,crs_deg=-1;
    int fix_q=0, valid_rmc=0, sats_used_gga=0;
    uint32_t last_status_ms = 0;

    for(;;){
        fd_set rf; struct timeval tv={0,100000};
        FD_ZERO(&rf); FD_SET(fd,&rf);
        if(select(fd+1,&rf,NULL,NULL,&tv)<0 && errno!=EINTR){perror("select");break;}

        int pos_updated=0;          /* we will send GPI */
        int sat_updated=0;          /* satellite table changed */

        /* ------- read all available serial ------------------------------ */
        if(FD_ISSET(fd,&rf)){
            ssize_t n=read(fd,buf+len,sizeof(buf)-1-len);
            if(n>0){len+=n; buf[len]=0;}

            char *p=buf,*eol;
            while((eol=strchr(p,'\n'))){
                *eol=0;
                if(verbose>1) fprintf(stderr,"NMEA: %s\n",p);

                /* ---- $GPRMC (drive position output) ---- */
                if(!strncmp(p,"$GPRMC",6)){
                    char *t[13]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<13; q=strtok(NULL,",")) t[i++]=q;
                    if(i>=9){
                        valid_rmc = (t[2][0]=='A');
                        if(valid_rmc){
                            lat = dm_to_deg(atof(t[3]));
                            lon = dm_to_deg(atof(t[5]));
                            if(t[4][0]=='S') lat=-lat;
                            if(t[6][0]=='W') lon=-lon;
                        }
                        spd_kn  = atof(t[7]);
                        crs_deg = atof(t[8]);
                        pos_updated = 1;          /* ONLY RMC triggers GPI */
                    }
                }
                /* ---- $GPGGA (altitude / fix quality) ---- */
                else if(!strncmp(p,"$GPGGA",6)){
                    char *t[15]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<15; q=strtok(NULL,",")) t[i++]=q;
                    if(i>=10){
                        fix_q        = atoi(t[6]);
                        sats_used_gga= atoi(t[7]);
                        alt          = atof(t[9]);
                        /* DOES NOT set pos_updated */
                    }
                }
                /* ---- $GPGSV (sat list; send once per burst) ---- */
                else if(!strncmp(p,"$GPGSV",6)){
                    char *t[72]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<72; q=strtok(NULL,",")) t[i++]=q;
                    if(i>=4){
                        int total = atoi(t[1]);
                        int index = atoi(t[2]);
                        sats_vis_gsv = atoi(t[3]);
                        for(int idx=4; idx+3<i && idx<72; idx+=4){
                            if(!t[idx]||!strlen(t[idx])) continue;
                            uint8_t prn=atoi(t[idx]);
                            int si=idx_for_prn(prn); if(si<0) continue;
                            sat_elev[si]= t[idx+1]?atoi(t[idx+1]):0;
                            sat_az  [si]= t[idx+2]?atoi(t[idx+2]):0;
                            sat_snr [si]= t[idx+3]?atoi(t[idx+3]):0;
                        }
                        if(index == total) sat_updated = 1; /* only on last GSV */
                    }
                }
                /* ---- $GPGSA (used-in-fix list) ---- */
                else if(!strncmp(p,"$GPGSA",6)){
                    memset(sat_used,0,sizeof(sat_used));
                    char *t[20]={0}; int i=0;
                    for(char *q=strtok(p,","); q&&i<20; q=strtok(NULL,",")) t[i++]=q;
                    for(int s=3; s<15 && s<i; s++){
                        if(t[s] && strlen(t[s])){
                            uint8_t prn=atoi(t[s]);
                            int si=idx_for_prn(prn);
                            if(si>=0) sat_used[si]=1;
                        }
                    }
                    sat_updated = 1;
                }
                p=eol+1;
            }
            memmove(buf,p,len-(p-buf)); len -= (p-buf);
        }

        uint32_t now_ms = millis();

        /* ---- 1 Hz-max GPS_STATUS -------------------------------------- */
        if(sat_updated && now_ms - last_status_ms >= 1000){
            uint8_t vis = sats_vis_gsv ? sats_vis_gsv : sats_used_gga;
            uint8_t frame[128]; size_t sz;
            sz = pack_gps_status(frame, vis,
                                 sat_prn, sat_used,
                                 sat_elev, sat_az, sat_snr);
            sendto(sock,frame,sz,0,(struct sockaddr*)&dst,sizeof(dst));
            last_status_ms = now_ms;
            sat_updated = 0;               /* table transmitted */
        }

        /* ---- send position only on valid fix and RMC update ------------ */
        if(!pos_updated) continue;
        int good = (valid_rmc || fix_q);
        if(!good) continue;

        double spd_ms  = spd_kn*0.514444;
        double crs_rad = crs_deg*M_PI/180.0;
        int32_t lat_e7 = (int32_t)(lat*1e7);
        int32_t lon_e7 = (int32_t)(lon*1e7);
        int32_t alt_mm = (int32_t)(alt*1000.0);
        int16_t vx     = (int16_t)(spd_ms*cos(crs_rad)*100.0);
        int16_t vy     = (int16_t)(spd_ms*sin(crs_rad)*100.0);
        uint16_t hdg   = (crs_deg>=0)?(uint16_t)(crs_deg*100):UINT16_MAX;

        uint8_t frame[128]; size_t sz;
        sz = pack_gpi(frame, now_ms,
                      lat_e7, lon_e7,
                      alt_mm, alt_mm,
                      vx, vy, 0, hdg);
        sendto(sock,frame,sz,0,(struct sockaddr*)&dst,sizeof(dst));

        /* piggy-back a GPS_STATUS if more than 1 s has passed */
        if(now_ms - last_status_ms >= 1000){
            uint8_t vis = sats_vis_gsv ? sats_vis_gsv : sats_used_gga;
            sz = pack_gps_status(frame, vis,
                                 sat_prn, sat_used,
                                 sat_elev, sat_az, sat_snr);
            sendto(sock,frame,sz,0,(struct sockaddr*)&dst,sizeof(dst));
            last_status_ms = now_ms;
        }

        if(verbose){
            int used_cnt=0; for(int i=0;i<20;i++) if(sat_used[i]) used_cnt++;
            printf("t=%u ms  fix=%s  lat=%.7f lon=%.7f alt=%.1f m  "
                   "spd=%.2f kn crs=%.1f°  sats: vis=%d used=%d\n",
                   now_ms, valid_rmc?"A":"GGA",
                   lat, lon, alt,
                   spd_kn, crs_deg,
                   sats_vis_gsv ? sats_vis_gsv : sats_used_gga,
                   used_cnt);
            fflush(stdout);
        }

        valid_rmc=0;           /* wait for next RMC line */
    }
    return 0;
}
