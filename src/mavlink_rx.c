/*  mavlink_rx.c  —  MAVLink sniffer + RC-triggered commands
    Build: gcc -O2 -Wall -Wextra -std=c11 -o mavlink_rx mavlink_rx.c
*/

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ----------------------------------------------------------------- const */
#define DEF_DEV  "/dev/ttyUSB0"
#define DEF_BAUD 460800
#define DEF_PER  1000                       /* stats period ms */

#define STX_V1 0xFEu
#define STX_V2 0xFDu

#define ID_RC_OVERRIDE   70u
#define ID_RADIO_STATUS 109u
#define LEN_RC_MIN      34u                 /* 16 channels + sys/comp */
#define LEN_RAD_MIN      8u
#define MAX_PAY        255u

#define SCRIPT_PATH "/usr/bin/channels.sh"

/* ---------------------------------------------------------------- helpers */
static inline uint64_t ms_now(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u;
}
static void die(const char *m) { perror(m); exit(1); }

/* ---------------------------------------------------------------- frame */
typedef struct {
    uint8_t  ver, len, sysid, compid;
    uint32_t msgid;
    uint8_t  pay[MAX_PAY];
} frame_t;

/* ---------------------------------------------------------------- parser */
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
    /* -- MAVLink-1 -- */
    case V1_LEN:  cur.len = b; idx = 0;        st = V1_SEQ;  break;
    case V1_SEQ:                              st = V1_SYS;  break;
    case V1_SYS:  cur.sysid  = b;             st = V1_COMP; break;
    case V1_COMP: cur.compid = b;             st = V1_ID;   break;
    case V1_ID:   cur.msgid  = b;             st = cur.len ? V1_PAY : V1_C1; break;
    case V1_PAY:
        if (idx < (int)MAX_PAY) cur.pay[idx] = b;
        if (++idx == (int)cur.len) st = V1_C1;
        break;
    case V1_C1: st = V1_C2; break;
    case V1_C2: *out = cur; st = IDLE; return true;
    /* -- MAVLink-2 -- */
    case V2_LEN:  cur.len = b;               st = V2_INCF; break;
    case V2_INCF: incf = b;                  st = V2_COMPF;break;
    case V2_COMPF:                            st = V2_SEQ; break;
    case V2_SEQ:                              st = V2_SYS; break;
    case V2_SYS:  cur.sysid  = b;            st = V2_COMP; break;
    case V2_COMP: cur.compid = b;            st = V2_ID0;  break;
    case V2_ID0:  cur.msgid  = b;            st = V2_ID1;  break;
    case V2_ID1:  cur.msgid |= (uint32_t)b << 8;  st = V2_ID2; break;
    case V2_ID2:  cur.msgid |= (uint32_t)b << 16; idx = 0;
                  st = cur.len ? V2_PAY : V2_C1;  break;
    case V2_PAY:
        if (idx < (int)MAX_PAY) cur.pay[idx] = b;
        if (++idx == (int)cur.len) st = V2_C1;
        break;
    case V2_C1: st = V2_C2; break;
    case V2_C2:
        if (incf & 1) { sig_left = 13; st = V2_SIG; }
        else          { *out = cur; st = IDLE; return true; }
        break;
    case V2_SIG:
        if (--sig_left == 0) { *out = cur; st = IDLE; return true; }
        break;
    }
    return false;
}

/* ---------------------------------------------------------------- trigger */
typedef struct {
    int last;
    bool pending;
    int old_val;
    uint64_t start_ms;
} ch_state_t;

/* ---------------------------------------------------------------- stats */
typedef struct {
    uint32_t period; uint64_t next_ms;
    uint32_t frames, bytes, rc_cnt, radio_cnt;
    uint8_t  rc_sys, rc_comp;
    uint16_t rc[16];
    uint8_t  rssi, remrssi, txbuf, noise;
    struct { int ch, old_v, new_v, rt; } execs[32];
    int exec_cnt;
} stats_t;
static void stat_reset(stats_t *s)
{
    memset(&s->frames, 0, sizeof(stats_t) - offsetof(stats_t, frames));
}

/* ---------------------------------------------------------------- CLI */
typedef struct {
    const char *dev; int baud; uint32_t period; bool raw;
    bool cmd_en; int min_delta; uint32_t persist_ms, pause_ms;
    uint16_t chan_mask;
} cfg_t;

static void cfg_default(cfg_t *c)
{
    c->dev = DEF_DEV; c->baud = DEF_BAUD; c->period = DEF_PER; c->raw = false;
    c->cmd_en = false; c->min_delta = 100; c->persist_ms = 500; c->pause_ms = 500;
    c->chan_mask = 0xFFFF;
}
static void parse_mask(cfg_t *c, const char *s)
{
    c->chan_mask = 0;
    while (*s) {
        int n = strtol(s, (char **)&s, 10);
        if (n < 1 || n > 16) die("channel");
        c->chan_mask |= 1u << (n - 1);
        if (*s == ',') ++s;
    }
}
static void parse_cli(int ac, char **av, cfg_t *c)
{
    static const struct option L[] = {
        {"device", 1, 0, 'd'}, {"baud", 1, 0, 'b'}, {"period", 1, 0, 'p'},
        {"raw", 0, 0, 'r'}, {"command-parse", 0, 0, 'c'},
        {"min-change", 1, 0, 1}, {"persist", 1, 0, 2}, {"pause", 1, 0, 3},
        {"channels", 1, 0, 4}, {0}
    };
    int o, i;
    while ((o = getopt_long(ac, av, "d:b:p:r", L, &i)) != -1)
        switch (o) {
            case 'd': c->dev = optarg; break;
            case 'b': c->baud = atoi(optarg); break;
            case 'p': c->period = atoi(optarg); break;
            case 'r': c->raw = true; break;
            case 'c': c->cmd_en = true; break;
            case 1:   c->min_delta = atoi(optarg); break;
            case 2:   c->persist_ms = atoi(optarg); break;
            case 3:   c->pause_ms = atoi(optarg); break;
            case 4:   parse_mask(c, optarg); break;
            default:  fprintf(stderr, "usage: %s [opts]\n", av[0]); exit(1);
        }
}

/* ---------------------------------------------------------------- exec */
static int run_script(int ch, int val)
{
    pid_t p = fork();
    if (p == 0) {
        char cbuf[4], vbuf[8];
        snprintf(cbuf, sizeof cbuf, "%d", ch);
        snprintf(vbuf, sizeof vbuf, "%d", val);
        execl(SCRIPT_PATH, SCRIPT_PATH, cbuf, vbuf, (char *)NULL);
        _exit(127);
    }
    int st; waitpid(p, &st, 0);
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

/* ---------------------------------------------------------------- main */
int main(int argc, char **argv)
{
    cfg_t cfg; cfg_default(&cfg); parse_cli(argc, argv, &cfg);

    int fd = open(cfg.dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) die("open");
    struct termios t = {0}; tcgetattr(fd, &t); cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD; cfsetispeed(&t, B460800); cfsetospeed(&t, B460800);
    tcsetattr(fd, TCSANOW, &t);

    printf("Listening on %s @%d raw=%s cmd=%s period=%u\n",
           cfg.dev, cfg.baud, cfg.raw ? "on" : "off",
           cfg.cmd_en ? "on" : "off", cfg.period);

    struct pollfd pfd = {.fd = fd, .events = POLLIN};
    uint8_t buf[512]; frame_t f;
    stats_t st = {.period = cfg.period, .next_ms = ms_now() + cfg.period};
    stat_reset(&st);
    ch_state_t ch[16] = {0};
    uint64_t next_ok_exec = 0;

    while (1) {
        if (poll(&pfd, 1, 50) <= 0) continue;
        ssize_t n = read(fd, buf, sizeof buf); if (n <= 0) continue;
        st.bytes += (uint32_t)n;

        for (ssize_t i = 0; i < n; ++i) {
            if (!parse_byte(buf[i], &f)) continue;
            st.frames++;

            /* raw preview ------------------------------------------------ */
            if (cfg.raw) {
                printf("RAW v=%u id=%" PRIu32 " len=%u |", f.ver, f.msgid, f.len);
                uint8_t show = f.len > 32 ? 32 : f.len;
                for (uint8_t j = 0; j < show; ++j) printf(" %02X", f.pay[j]);
                if (f.len > show) printf(" …");
                printf("\n");
            }

            /* RC override ------------------------------------------------ */
            if (f.msgid == ID_RC_OVERRIDE && f.len >= LEN_RC_MIN) {
                st.rc_cnt++;

                /* channels are first; sys/comp are last two bytes */
                int sys_ofs = f.len - 2;
                st.rc_sys  = f.pay[sys_ofs];
                st.rc_comp = f.pay[sys_ofs + 1];

                const uint16_t *vals = (const uint16_t *)f.pay;
                for (int cix = 0; cix < 16; ++cix) st.rc[cix] = vals[cix];

                if (cfg.cmd_en) {
                    uint64_t now = ms_now();
                    for (int cix = 0; cix < 16; ++cix) {
                        if (!(cfg.chan_mask & (1u << cix))) continue;
                        int val = vals[cix];
                        ch_state_t *s = &ch[cix];
                        if (!s->last) s->last = val;
                        int diff = abs(val - s->last);

                        if (s->pending) {
                            if (diff < cfg.min_delta) { s->pending = false; continue; }
                            if (now - s->start_ms >= cfg.persist_ms) {
                                if (now >= next_ok_exec && st.exec_cnt < 32) {
                                    uint64_t t0 = ms_now();
                                    run_script(cix + 1, val);
                                    int rt = (int)(ms_now() - t0);

                                    st.execs[st.exec_cnt].ch     = cix + 1;
                                    st.execs[st.exec_cnt].old_v  = s->old_val;
                                    st.execs[st.exec_cnt].new_v  = val;
                                    st.execs[st.exec_cnt].rt     = rt;
                                    st.exec_cnt++;

                                    next_ok_exec = now + cfg.pause_ms;
                                    s->last = val;
                                }
                                s->pending = false;
                            }
                        } else if (diff >= cfg.min_delta) {
                            s->pending  = true;
                            s->old_val  = s->last;
                            s->start_ms = now;
                        }
                    }
                }
            }
            /* Radio status ----------------------------------------------- */
            else if (f.msgid == ID_RADIO_STATUS && f.len >= LEN_RAD_MIN) {
                st.radio_cnt++;
                const uint8_t *p = f.pay;
                st.rssi = p[4]; st.remrssi = p[5];
                st.txbuf = p[6]; st.noise = p[7];
            }
        }

        uint64_t now = ms_now();
        if (now >= st.next_ms) {
            printf("%" PRIu64 " MAVLINK_STATS %u:%u:%u:%u:0\n",
                   now, cfg.period, st.frames,
                   (st.rc_cnt ? 1 : 0) + (st.radio_cnt ? 1 : 0), st.bytes);

            if (st.rc_cnt) {
                printf("%" PRIu64 " RC_CHANNELS_OVERRIDE %u %u %u ",
                       now, st.rc_cnt, st.rc_sys, st.rc_comp);
                for (int i = 0; i < 16; ++i)
                    printf("%u%c", st.rc[i], i == 15 ? '\n' : ':');
            }
            if (st.radio_cnt) {
                printf("%" PRIu64 " RADIO_STATUS %u %u:%u:%u:%u\n",
                       now, st.radio_cnt,
                       st.rssi, st.remrssi, st.txbuf, st.noise);
            }
            for (int i = 0; i < st.exec_cnt; ++i) {
                printf("%" PRIu64 " MAVLINK_EXEC %d:%d:%d:%d:%d\n",
                       now, i + 1,
                       st.execs[i].ch, st.execs[i].old_v,
                       st.execs[i].new_v, st.execs[i].rt);
            }

            fflush(stdout);                      /* immediate flush */
            stat_reset(&st); st.next_ms += cfg.period;
        }
    }
}
