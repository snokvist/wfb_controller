/*  mavlink_rx.c  — MAVLink sniffer + RC-triggered commands + flexible UDP/UART forwarder
    Build: gcc -O2 -Wall -Wextra -std=c11 -o mavlink_rx mavlink_rx.c
*/

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* --------------------------------------------------------------- constants */
#define DEF_DEV  "/dev/ttyUSB0"
#define DEF_BAUD 460800
#define DEF_PER  1000                       /* stats period ms */

#define STX_V1 0xFEu
#define STX_V2 0xFDu

#define ID_RC_OVERRIDE          70u
#define ID_RADIO_STATUS        109u
#define ID_GLOBAL_POSITION_INT  33u
#define ID_GPS_STATUS           25u

#define LEN_RC_MIN       34u           /* 16 channels + sys/comp */
#define LEN_RAD_MIN       8u
#define LEN_GPI_MIN      28u
#define LEN_GPS_STATUS  101u
#define MAX_PAY        255u

#define SCRIPT_PATH "/usr/bin/channels.sh"

/* --------------------------------------------------------------- helpers */
static inline uint64_t ms_now(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u;
}
static void die(const char *m) { perror(m); exit(1); }

/* --------------------------------------------------------------- MAV frame */
typedef struct {
    uint8_t  ver, len, sysid, compid;
    uint32_t msgid;
    uint8_t  pay[MAX_PAY];
} frame_t;

/* --------------------------------------------------------------- parser FSM */
static bool parse_byte(uint8_t b, frame_t *out)
{
    enum { IDLE,
           V1_LEN, V1_SEQ, V1_SYS, V1_COMP, V1_ID, V1_PAY, V1_C1, V1_C2,
           V2_LEN, V2_INCF, V2_COMPF, V2_SEQ, V2_SYS, V2_COMP,
           V2_ID0, V2_ID1, V2_ID2, V2_PAY, V2_C1, V2_C2, V2_SIG };

    static int st = IDLE, idx = 0, sig_left = 0;
    static uint8_t incf = 0;
    static frame_t cur;

    switch (st) {
    case IDLE:
        if (b == STX_V1) { memset(&cur, 0, sizeof cur); cur.ver = 1; st = V1_LEN; }
        else if (b == STX_V2) { memset(&cur, 0, sizeof cur); cur.ver = 2; st = V2_LEN; }
        break;

/* MAVLink-1 */
    case V1_LEN:  cur.len = b; idx = 0;               st = V1_SEQ;  break;
    case V1_SEQ:                                      st = V1_SYS;  break;
    case V1_SYS:  cur.sysid  = b;                     st = V1_COMP; break;
    case V1_COMP: cur.compid = b;                     st = V1_ID;   break;
    case V1_ID:   cur.msgid  = b;                     st = cur.len ? V1_PAY : V1_C1; break;
    case V1_PAY:  if (idx < MAX_PAY) cur.pay[idx] = b;
                  if (++idx == cur.len) st = V1_C1; break;
    case V1_C1:   st = V1_C2; break;
    case V1_C2:   *out = cur; st = IDLE; return true;

/* MAVLink-2 */
    case V2_LEN:  cur.len = b;                        st = V2_INCF; break;
    case V2_INCF: incf = b;                           st = V2_COMPF;break;
    case V2_COMPF:                                    st = V2_SEQ;  break;
    case V2_SEQ:                                      st = V2_SYS;  break;
    case V2_SYS:  cur.sysid  = b;                     st = V2_COMP; break;
    case V2_COMP: cur.compid = b;                     st = V2_ID0;  break;
    case V2_ID0:  cur.msgid  = b;                     st = V2_ID1;  break;
    case V2_ID1:  cur.msgid |= (uint32_t)b << 8;      st = V2_ID2;  break;
    case V2_ID2:  cur.msgid |= (uint32_t)b << 16; idx = 0;
                  st = cur.len ? V2_PAY : V2_C1;       break;
    case V2_PAY:  if (idx < MAX_PAY) cur.pay[idx] = b;
                  if (++idx == cur.len) st = V2_C1;    break;
    case V2_C1:   st = V2_C2; break;
    case V2_C2:   if (incf & 1){ sig_left = 13; st = V2_SIG; }
                  else { *out = cur; st = IDLE; return true; } break;
    case V2_SIG:  if (--sig_left == 0){ *out = cur; st = IDLE; return true; } break;
    }
    return false;
}

/* --------------------------------------------------------------- triggers */
typedef struct {
    int last;
    bool pending;
    int old_val;
    uint64_t start_ms;
} ch_state_t;

/* --------------------------------------------------------------- statistics */
typedef struct {
    uint32_t period; uint64_t next_ms;
    uint32_t frames, bytes, rc_cnt, radio_cnt;
    uint32_t gpi_cnt, gpss_cnt;
    uint32_t udp_rx, udp_tx;

/* RC override */
    uint8_t  rc_sys, rc_comp;
    uint16_t rc[16];

/* radio status */
    uint8_t  rssi, remrssi, txbuf, noise;

/* GLOBAL_POSITION_INT */
    uint32_t gpi_time_ms;
    int32_t  lat, lon, alt, rel_alt;
    int16_t  vx, vy, vz;
    uint16_t hdg;

/* GPS_STATUS */
    uint8_t  sats_visible;
    uint8_t  prn[20], used[20], elev[20], azim[20], snr[20];

/* exec info */
    struct { int ch, old_v, new_v, rt; } execs[32];
    int exec_cnt;
} stats_t;

static void stat_reset(stats_t *s)
{
    memset(&s->frames, 0, sizeof(stats_t) - offsetof(stats_t, frames));
}

/* --------------------------------------------------------------- CLI / opts */
typedef struct {
    const char *dev; int baud; uint32_t period; bool raw;
    bool cmd_en; int min_delta; uint32_t persist_ms, pause_ms;
    uint16_t chan_mask;
    int udp_in_port;
    const char *udp_out_host, *udp_out_port;
    bool no_serial;
} cfg_t;

/* help */
static void print_help(const char *p)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  -d, --device PATH        Serial device (default %s)\n"
        "  -b, --baud RATE          Baud-rate   (default %d)\n"
        "  -n, --no-serial          Disable serial port\n"
        "  -u, --udp-in PORT        Listen for MAVLink on UDP PORT\n"
        "  -o, --udp-out HOST:PORT  Forward frames to HOST:PORT\n"
        "  -p, --period MS          Stats window in ms (default %u)\n"
        "  -r, --raw                Hex-dump each frame\n"
        "  -c, --command-parse      Enable RC-triggered scripts\n"
        "      --channels LIST      Comma list (1-16) to watch\n"
        "      --min-change VAL     Delta for trigger (default 100)\n"
        "      --persist MS         Value must persist (default 500)\n"
        "      --pause MS           Pause after exec (default 500)\n"
        "  -h, --help               Show this help\n",
        p, DEF_DEV, DEF_BAUD, DEF_PER);
}

static void cfg_default(cfg_t *c)
{
    c->dev = DEF_DEV; c->baud = DEF_BAUD; c->period = DEF_PER; c->raw = false;
    c->cmd_en = false; c->min_delta = 100; c->persist_ms = 500; c->pause_ms = 500;
    c->chan_mask = 0xFFFF;
    c->udp_in_port = 0; c->udp_out_host = c->udp_out_port = NULL;
    c->no_serial = false;
}
static void parse_mask(cfg_t *c, const char *s)
{
    c->chan_mask = 0;
    while (*s){
        int n = strtol(s, (char**)&s, 10);
        if (n < 1 || n > 16) die("channel list");
        c->chan_mask |= 1u << (n - 1);
        if (*s == ',') ++s;
    }
}
static void parse_udp_out(cfg_t *c, const char *s)
{
    const char *cpos = strchr(s, ':');
    if (!cpos) die("udp-out host:port");
    size_t hl = (size_t)(cpos - s);
    char *h = malloc(hl + 1); if (!h) die("malloc");
    memcpy(h, s, hl); h[hl] = 0;
    c->udp_out_host = h; c->udp_out_port = cpos + 1;
}

static void parse_cli(int ac, char **av, cfg_t *c)
{
    static const struct option L[] = {
        {"device",1,0,'d'}, {"baud",1,0,'b'}, {"period",1,0,'p'},
        {"raw",0,0,'r'}, {"command-parse",0,0,'c'},
        {"udp-in",1,0,'u'}, {"udp-out",1,0,'o'}, {"no-serial",0,0,'n'},
        {"help",0,0,'h'},
        {"min-change",1,0,1}, {"persist",1,0,2}, {"pause",1,0,3},
        {"channels",1,0,4}, {0}
    };
    int o, idx;
    while ((o = getopt_long(ac, av, "d:b:p:rcu:o:nh", L, &idx)) != -1)
        switch (o){
        case 'd': c->dev = optarg; break;
        case 'b': c->baud = atoi(optarg); break;
        case 'p': c->period = atoi(optarg); break;
        case 'r': c->raw = true; break;
        case 'c': c->cmd_en = true; break;
        case 'u': c->udp_in_port = atoi(optarg); break;
        case 'o': parse_udp_out(c, optarg); break;
        case 'n': c->no_serial = true; break;
        case 'h': print_help(av[0]); exit(0);
        case 1:   c->min_delta = atoi(optarg); break;
        case 2:   c->persist_ms = atoi(optarg); break;
        case 3:   c->pause_ms = atoi(optarg); break;
        case 4:   parse_mask(c, optarg); break;
        default:  print_help(av[0]); exit(1);
        }

    if (c->no_serial) c->dev = NULL;
    if (!c->dev && !c->udp_in_port && !c->udp_out_host){
        fprintf(stderr,"Need serial, udp-in or udp-out\n"); exit(1);
    }
}

/* --------------------------------------------------------------- serial */
static int open_serial(const cfg_t *cfg)
{
    for (;;){
        int fd = open(cfg->dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd >= 0){
            struct termios t={0};
            if (tcgetattr(fd,&t)==0){
                cfmakeraw(&t); t.c_cflag |= CLOCAL | CREAD;
                cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
                if (tcsetattr(fd,TCSANOW,&t)==0) return fd;
            }
            close(fd);
        }
        perror("serial"); fprintf(stderr,"Retrying in 3 s …\n"); sleep(3);
    }
}

/* --------------------------------------------------------------- UDP */
static int mk_nonblock(int fd)
{
    int fl = fcntl(fd,F_GETFL,0);
    if (fl<0 || fcntl(fd,F_SETFL,fl|O_NONBLOCK)<0) die("fcntl");
    return fd;
}
static int open_udp_in(int p)
{
    int fd = socket(AF_INET,SOCK_DGRAM,0); if (fd<0) die("sock in");
    int yes=1; setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes);
    struct sockaddr_in a={.sin_family=AF_INET,.sin_addr.s_addr=htonl(INADDR_ANY),
                          .sin_port=htons(p)};
    if (bind(fd,(struct sockaddr*)&a,sizeof a)<0) die("bind");
    return mk_nonblock(fd);
}
static int open_udp_out(const char*host,const char*port)
{
    struct addrinfo h={.ai_family=AF_INET,.ai_socktype=SOCK_DGRAM},*res;
    int r=getaddrinfo(host,port,&h,&res);
    if (r){ fprintf(stderr,"udp-out %s\n",gai_strerror(r)); exit(1); }
    int fd=socket(res->ai_family,res->ai_socktype,res->ai_protocol);
    if (fd<0) die("sock out");
    if (connect(fd,res->ai_addr,res->ai_addrlen)<0) die("connect");
    freeaddrinfo(res); return mk_nonblock(fd);
}

/* --------------------------------------------------------------- script exec */
static int run_script(int ch,int val)
{
    pid_t p=fork();
    if (!p){
        char c[4], v[8]; snprintf(c,sizeof c,"%d",ch); snprintf(v,sizeof v,"%d",val);
        execl(SCRIPT_PATH,SCRIPT_PATH,c,v,(char*)NULL); _exit(127);
    }
    int st; waitpid(p,&st,0);
    return WIFEXITED(st)?WEXITSTATUS(st):-1;
}

/* --------------------------------------------------------------- main */
int main(int argc,char**argv)
{
    cfg_t cfg; cfg_default(&cfg); parse_cli(argc,argv,&cfg);

    int fd_serial=-1;
    if (cfg.dev) fd_serial=open_serial(&cfg);
    int fd_udp_in=-1;
    if (cfg.udp_in_port) fd_udp_in=open_udp_in(cfg.udp_in_port);
    int fd_udp_out=-1;
    if (cfg.udp_out_host) fd_udp_out=open_udp_out(cfg.udp_out_host,cfg.udp_out_port);

    printf("Listening %sserial%s UDP-in=%s UDP-out=%s raw=%s cmd=%s period=%u\n",
           cfg.dev?"": "no-", cfg.dev?cfg.dev:"",
           cfg.udp_in_port ?"ON":"OFF",
           cfg.udp_out_host?"ON":"OFF",
           cfg.raw?"on":"off", cfg.cmd_en?"on":"off", cfg.period);

    uint8_t buf[2048]; frame_t f;
    stats_t st={.period=cfg.period,.next_ms=ms_now()+cfg.period};
    stat_reset(&st);
    ch_state_t ch[16]={0}; uint64_t next_ok_exec=0;

    for (;;){
        struct pollfd pf[3]; int nf=0;
        if (fd_serial>=0) pf[nf++] = (struct pollfd){fd_serial,POLLIN,0};
        if (fd_udp_in>=0) pf[nf++] = (struct pollfd){fd_udp_in ,POLLIN,0};
        int pr=poll(pf,nf,50);
        if (pr<0 && errno!=EINTR) pr=0;

        /* ---------- RX ---------- */
        for (int i=0;i<nf;++i) if (pf[i].revents&POLLIN){
            int fd = pf[i].fd;
            ssize_t n = (fd==fd_serial)?read(fd,buf,sizeof buf)
                                       :recv(fd,buf,sizeof buf,0);
            if (n<=0){
                if (fd==fd_serial && (n==0||(errno!=EAGAIN&&errno!=EINTR))){
                    perror("serial read"); close(fd_serial); fd_serial=-1; sleep(3);
                    if (cfg.dev) fd_serial=open_serial(&cfg);
                }
                continue;
            }
            st.bytes+=(uint32_t)n;
            if (fd==fd_udp_in) st.udp_rx+=(uint32_t)n;
            if (fd==fd_udp_in && fd_serial>=0) write(fd_serial,buf,n);
            if (fd_udp_out>=0 && n>0){
                ssize_t w=send(fd_udp_out,buf,n,0);
                if (w>0) st.udp_tx+=(uint32_t)w;
            }

            for (ssize_t ix=0; ix<n; ++ix) if (parse_byte(buf[ix],&f)){
                st.frames++;

                switch (f.msgid){
                case ID_RC_OVERRIDE: {
                    if (f.len<LEN_RC_MIN) break;
                    st.rc_cnt++; int so=f.len-2;
                    st.rc_sys=f.pay[so]; st.rc_comp=f.pay[so+1];
                    const uint16_t*vals=(const uint16_t*)f.pay;
                    for (int c=0;c<16;++c) st.rc[c]=vals[c];

                    if (cfg.cmd_en){
                        uint64_t now=ms_now();
                        for (int c=0;c<16;++c) if (cfg.chan_mask&(1u<<c)){
                            int val=vals[c]; ch_state_t*s=&ch[c];
                            if (!s->last) s->last=val;
                            int diff=abs(val-s->last);

                            if (s->pending){
                                if (diff<cfg.min_delta){ s->pending=false; continue; }
                                if (now-s->start_ms>=cfg.persist_ms &&
                                    now>=next_ok_exec && st.exec_cnt<32){
                                    uint64_t t0=ms_now();
                                    run_script(c+1,val);
                                    int rt=(int)(ms_now()-t0);
                                    st.execs[st.exec_cnt++] =
                                        (typeof(st.execs[0])){c+1,s->old_val,val,rt};
                                    next_ok_exec=now+cfg.pause_ms;
                                    s->last=val; s->pending=false;
                                }
                            } else if (diff>=cfg.min_delta){
                                s->pending=true; s->old_val=s->last; s->start_ms=now;
                            }
                        }
                    }
                    break;
                }
                case ID_RADIO_STATUS:
                    if (f.len<LEN_RAD_MIN) break;
                    st.radio_cnt++;
                    st.rssi=f.pay[4]; st.remrssi=f.pay[5];
                    st.txbuf=f.pay[6]; st.noise=f.pay[7];
                    break;

                case ID_GLOBAL_POSITION_INT:
                    if (f.len<LEN_GPI_MIN) break;
                    st.gpi_cnt++;
                    st.gpi_time_ms=*(uint32_t*)(f.pay+0);
                    st.lat        =*(int32_t *)(f.pay+4);
                    st.lon        =*(int32_t *)(f.pay+8);
                    st.alt        =*(int32_t *)(f.pay+12);
                    st.rel_alt    =*(int32_t *)(f.pay+16);
                    st.vx         =*(int16_t *)(f.pay+20);
                    st.vy         =*(int16_t *)(f.pay+22);
                    st.vz         =*(int16_t *)(f.pay+24);
                    st.hdg        =*(uint16_t*)(f.pay+26);
                    break;

                case ID_GPS_STATUS:
                    if (f.len<1) break;
                    st.gpss_cnt++;
                    memset(&st.prn, 0, sizeof st.prn);   /* clear slots */
                    st.sats_visible=f.pay[0];
                    if (f.len>=LEN_GPS_STATUS){
                        memcpy(st.prn ,f.pay+1 ,20);
                        memcpy(st.used,f.pay+21,20);
                        memcpy(st.elev,f.pay+41,20);
                        memcpy(st.azim,f.pay+61,20);
                        memcpy(st.snr ,f.pay+81,20);
                    }else{
                        size_t avail=f.len-1, chunk=avail/5; if(chunk>20)chunk=20;
                        const uint8_t*ptr=f.pay+1;
                        memcpy(st.prn ,ptr,chunk); ptr+=chunk;
                        memcpy(st.used,ptr,chunk); ptr+=chunk;
                        memcpy(st.elev,ptr,chunk); ptr+=chunk;
                        memcpy(st.azim,ptr,chunk); ptr+=chunk;
                        memcpy(st.snr ,ptr,chunk);
                    }
                    break;

                default: break;
                }

                if (cfg.raw){
                    printf("RAW v=%u id=%"PRIu32" len=%u |",f.ver,f.msgid,f.len);
                    uint8_t show=f.len>32?32:f.len;
                    for (uint8_t j=0;j<show;++j) printf(" %02X",f.pay[j]);
                    if (f.len>show) printf(" …"); puts("");
                }
            }
        }

        /* ---------- periodic stats ---------- */
        uint64_t now=ms_now();
        if (now>=st.next_ms){
            printf("%"PRIu64" MAVLINK_STATS %u:%u:%u:%u:%u:%u:%u:%u\n",
                   now,cfg.period,st.frames,
                   (st.rc_cnt?1:0)+(st.radio_cnt?1:0)+(st.gpi_cnt?1:0)+(st.gpss_cnt?1:0),
                   st.bytes,st.udp_rx,st.udp_tx,st.gpi_cnt,st.gpss_cnt);

            if (st.rc_cnt){
                printf("%"PRIu64" RC_CHANNELS_OVERRIDE %u %u %u ",now,st.rc_cnt,st.rc_sys,st.rc_comp);
                for (int i=0;i<16;++i) printf("%u%c",st.rc[i],i==15?'\n':':');
            }
            if (st.radio_cnt){
                printf("%"PRIu64" RADIO_STATUS %u %u:%u:%u:%u\n",
                       now,st.radio_cnt,st.rssi,st.remrssi,st.txbuf,st.noise);
            }
            if (st.gpi_cnt){
                printf("%"PRIu64" GLOBAL_POSITION_INT %u %"PRIu32" %"
                       PRId32":%"PRId32":%"PRId32":%"PRId32":%"
                       PRId16":%"PRId16":%"PRId16":%"PRIu16"\n",
                       now,st.gpi_cnt,st.gpi_time_ms,
                       st.lat,st.lon,st.alt,st.rel_alt,
                       st.vx,st.vy,st.vz,st.hdg);
            }
            if (st.gpss_cnt){
                printf("%"PRIu64" GPS_STATUS %u %u ",now,st.gpss_cnt,st.sats_visible);
                for (int i=0;i<20;++i){
                    printf("%u:%u:%u:%u:%u%c",
                           st.prn[i],st.used[i],st.elev[i],st.azim[i],st.snr[i],
                           i==19?'\n':'|');
                }
            }
            for (int i=0;i<st.exec_cnt;++i){
                printf("%"PRIu64" MAVLINK_EXEC %d:%d:%d:%d:%d\n",
                       now,i+1,st.execs[i].ch,st.execs[i].old_v,
                       st.execs[i].new_v,st.execs[i].rt);
            }

            fflush(stdout);
            stat_reset(&st); st.next_ms+=cfg.period;
        }
    }
}
