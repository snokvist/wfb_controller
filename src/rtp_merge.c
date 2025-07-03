/*
 * rtp_merge.c — RTP duplicate-stream merger
 *               · bitmap dedup (4 k-pkt sliding window, O(1))
 *               · batched RX  via recvmmsg  (--batch=N  , default 16)
 *               · batched TX  via sendmmsg  (same N, single connected peer)
 *               · optional CPU pin (--cpu=N)
 *               · per-port signed dloss = agg_fwd − port_recv
 *               · soft-realtime SCHED_FIFO 50
 *
 * Build:
 *   gcc -O3 -march=native -Wall -std=gnu11 -o rtp_merge rtp_merge.c
 *
 * Example:
 *   sudo setcap cap_sys_nice=eip ./rtp_merge 127.0.0.1 5600 \
 *        --cpu=3 --batch=32 5702 5599
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <time.h>
#include <sched.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <linux/version.h>

#ifndef recvmmsg
#  define recvmmsg(sockfd, msgvec, vlen, flags, timeout) \
         syscall(SYS_recvmmsg, sockfd, msgvec, vlen, flags, timeout)
#endif
#ifndef sendmmsg
#  define sendmmsg(sockfd, msgvec, vlen, flags) \
         syscall(SYS_sendmmsg, sockfd, msgvec, vlen, flags)
#endif

/* ----------------------------------------------------------------- tunables */
#define MAX_SOCKS  16
#define MAX_PKT    1500
#define WIN_BITS   4096
#define WIN_MASK   (WIN_BITS - 1)
#define MAX_BATCH  64

/* ----------------------------------------------------------------- helpers */
static void try_rt(int prio)
{
    struct sched_param sp = { .sched_priority = prio };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == 0)
        fprintf(stderr, "◎ SCHED_FIFO %d enabled\n", prio);
    else
        perror("sched_setscheduler");
}

static void pin_cpu(int cpu)
{
    if (cpu < 0) return;
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) == 0)
        fprintf(stderr, "◎ Pinned to CPU %d\n", cpu);
    else
        perror("sched_setaffinity");
}

static inline int16_t seq_diff(uint16_t a, uint16_t b)
{   return (int16_t)(a - b); }

/* ----------------------------------------------------------------- bitmap dedup */
static uint64_t bm[WIN_BITS / 64];
static uint16_t win_start;

static void dedup_reset(uint16_t start_seq)
{
    memset(bm, 0, sizeof(bm));
    win_start = start_seq;
}

static bool dedup_seen(uint16_t seq)
{
    int16_t diff = seq_diff(seq, win_start);
    if (diff < 0) return true;

    if (diff >= WIN_BITS) {                        /* slide window */
        uint32_t shift = diff - WIN_BITS + 1;
        while (shift--) {
            uint32_t idx = win_start & WIN_MASK;
            bm[idx >> 6] &= ~(1ULL << (idx & 63));
            win_start++;
        }
        diff = seq_diff(seq, win_start);
    }

    uint32_t idx = (win_start + diff) & WIN_MASK;
    uint64_t *word = &bm[idx >> 6];
    uint64_t  mask = 1ULL << (idx & 63);

    if (*word & mask) return true;                 /* duplicate */
    *word |= mask;                                 /* mark seen */
    return false;
}

/* ----------------------------------------------------------------- sockets */
static int make_sock(int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket"); exit(EXIT_FAILURE); }

    int reuse = 1, sz = 256 * 1024;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(s, SOL_SOCKET, SO_RCVBUF , &sz   , sizeof(sz));

    struct sockaddr_in a = {0};
    a.sin_family      = AF_INET;
    a.sin_port        = htons(port);
    a.sin_addr.s_addr = INADDR_ANY;

    if (bind(s, (struct sockaddr *)&a, sizeof(a)) < 0) {
        perror("bind"); exit(EXIT_FAILURE);
    }
    return s;
}

/* ----------------------------------------------------------------- per-input counters */
typedef struct {
    int       sock, port;
    uint64_t  recv, fwd, dup, gaps, late;
} input_t;

/* ----------------------------------------------------------------- main */
int main(int argc, char *argv[])
{
    if (argc < 4) {
        fprintf(stderr,
        "Usage: %s OUT_IP OUT_PORT [--batch=N|-bN] [--cpu=N|-cN] IN_PORT...\n",
        argv[0]); return EXIT_FAILURE; }

    const char *out_ip   = argv[1];
    int         out_port = atoi(argv[2]);

    int batch = 16, cpu_pin = -1;
    int argi  = 3;
    while (argi < argc && argv[argi][0] == '-') {
        if      (!strncmp(argv[argi], "--batch=", 8)) batch   = atoi(argv[argi]+8);
        else if (!strncmp(argv[argi], "-b", 2))       batch   = atoi(argv[argi]+2);
        else if (!strncmp(argv[argi], "--cpu=", 6))   cpu_pin = atoi(argv[argi]+6);
        else if (!strncmp(argv[argi], "-c", 2))       cpu_pin = atoi(argv[argi]+2);
        else { fprintf(stderr, "Unknown option %s\n", argv[argi]); return EXIT_FAILURE; }
        argi++;
    }
    batch = (batch < 1) ? 1 : (batch > MAX_BATCH ? MAX_BATCH : batch);

    int n_in = argc - argi;
    if (n_in < 1 || n_in > MAX_SOCKS) {
        fprintf(stderr, "Must supply 1–%d IN_PORTs\n", MAX_SOCKS);
        return EXIT_FAILURE;
    }

    try_rt(50);
    pin_cpu(cpu_pin);

    /* create input sockets */
    input_t in[MAX_SOCKS] = {0};
    for (int i = 0; i < n_in; i++) {
        in[i].port = atoi(argv[argi + i]);
        in[i].sock = make_sock(in[i].port);
    }

    /* output socket (connected) */
    int out_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (out_sock < 0) { perror("socket"); return EXIT_FAILURE; }
    int sz = 256 * 1024;
    setsockopt(out_sock, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));

    struct sockaddr_in out_addr = {0};
    out_addr.sin_family = AF_INET;
    out_addr.sin_port   = htons(out_port);
    if (inet_pton(AF_INET, out_ip, &out_addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP %s\n", out_ip); return EXIT_FAILURE;
    }
    if (connect(out_sock, (struct sockaddr *)&out_addr, sizeof(out_addr)) < 0) {
        perror("connect"); return EXIT_FAILURE;
    }

    /* shared RX buffers */
    static uint8_t  buf[MAX_BATCH][MAX_PKT];
    static struct   sockaddr_in addrs[MAX_BATCH];
    static struct   iovec  rx_iov[MAX_BATCH];
    static struct   mmsghdr rx_msg[MAX_BATCH];
    for (int i = 0; i < MAX_BATCH; i++) {
        rx_iov[i].iov_base = buf[i];
        rx_iov[i].iov_len  = MAX_PKT;
        rx_msg[i].msg_hdr.msg_iov    = &rx_iov[i];
        rx_msg[i].msg_hdr.msg_iovlen = 1;
        rx_msg[i].msg_hdr.msg_name   = &addrs[i];
        rx_msg[i].msg_hdr.msg_namelen= sizeof(addrs[i]);
    }

    /* TX batch buffers */
    static struct   iovec  tx_iov[MAX_BATCH];
    static struct   mmsghdr tx_msg[MAX_BATCH];
    for (int i = 0; i < MAX_BATCH; i++) {
        tx_msg[i].msg_hdr.msg_name    = NULL;   /* connected socket */
        tx_msg[i].msg_hdr.msg_namelen = 0;
    }

    /* running state */
    dedup_reset(0);
    uint32_t last_ssrc = 0;
    uint16_t last_seq  = 0;
    bool     last_valid= false;

    uint64_t agg_recv=0, agg_fwd=0, agg_dup=0, agg_gap=0, agg_late=0;
    struct timespec t_last; clock_gettime(CLOCK_MONOTONIC, &t_last);

    /* ----------------------------------------------------------------- main loop */
    for (;;) {
        /* poll up to 1 s */
        fd_set rfds; FD_ZERO(&rfds); int maxfd = -1;
        for (int i = 0; i < n_in; i++) {
            FD_SET(in[i].sock, &rfds);
            if (in[i].sock > maxfd) maxfd = in[i].sock;
        }
        struct timeval tv = {1,0};
        if (select(maxfd+1, &rfds, NULL, NULL, &tv) < 0) {
            if (errno == EINTR) continue;
            perror("select"); return EXIT_FAILURE;
        }

        /* per ready socket */
        for (int i = 0; i < n_in; i++) {
            if (!FD_ISSET(in[i].sock, &rfds)) continue;

            int tx_cnt = 0;
            int got;
            do {
                got = recvmmsg(in[i].sock, rx_msg, batch, MSG_DONTWAIT, NULL);
                if (got < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    perror("recvmmsg"); break;
                }

                for (int j = 0; j < got; j++) {
                    size_t len = rx_msg[j].msg_len;
                    if (len < 12) continue;

                    uint8_t *p = buf[j];
                    uint16_t seq  = (p[2] << 8) | p[3];
                    uint32_t ssrc = (p[8] << 24) | (p[9] << 16) |
                                    (p[10] << 8) | p[11];

                    in[i].recv++;  agg_recv++;

                    if (ssrc != last_ssrc) {            /* stream reset */
                        last_ssrc = ssrc;
                        dedup_reset(seq);
                        last_valid = false;
                    }

                    if (dedup_seen(seq)) {              /* duplicate */
                        in[i].dup++;  agg_dup++;
                        continue;
                    }

                    if (last_valid) {                   /* gap / late */
                        int16_t d = seq_diff(seq, last_seq + 1);
                        if (d > 0)      { in[i].gaps += d; agg_gap  += d; }
                        else if (d < 0) { in[i].late++;    agg_late++;   }
                    }
                    last_seq = seq; last_valid = true;

                    /* queue packet for batched TX */
                    tx_iov[tx_cnt].iov_base = p;
                    tx_iov[tx_cnt].iov_len  = len;
                    tx_msg[tx_cnt].msg_hdr.msg_iov    = &tx_iov[tx_cnt];
                    tx_msg[tx_cnt].msg_hdr.msg_iovlen = 1;
                    tx_cnt++;

                    in[i].fwd++;  agg_fwd++;

                    if (tx_cnt == batch) {
                        if (sendmmsg(out_sock, tx_msg, tx_cnt, 0) < 0)
                            perror("sendmmsg");
                        tx_cnt = 0;
                    }
                }
            } while (got == batch);

            if (tx_cnt > 0) {                          /* flush remainder */
                if (sendmmsg(out_sock, tx_msg, tx_cnt, 0) < 0)
                    perror("sendmmsg");
            }
        }

        /* once per second print stats */
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - t_last.tv_sec) +
                         (now.tv_nsec - t_last.tv_nsec) / 1e9;
        if (elapsed >= 1.0) {
            double ts = now.tv_sec + now.tv_nsec / 1e9;

            for (int i = 0; i < n_in; i++) {
                int64_t dloss = (int64_t)agg_fwd - (int64_t)in[i].recv;
                printf("%.3f:port=%d:recv=%"PRIu64":fwd=%"PRIu64":dupes=%"PRIu64
                       ":gaps=%"PRIu64":late=%"PRIu64":dloss=%"PRId64"\n",
                       ts, in[i].port, in[i].recv, in[i].fwd, in[i].dup,
                       in[i].gaps, in[i].late, dloss);
                in[i].recv = in[i].fwd = in[i].dup = 0;
                in[i].gaps = in[i].late = 0;
            }

            printf("%.3f:agg:recv=%"PRIu64":fwd=%"PRIu64":dupes=%"PRIu64
                   ":gaps=%"PRIu64":late=%"PRIu64":ssrc=0x%08X\n",
                   ts, agg_recv, agg_fwd, agg_dup, agg_gap, agg_late, last_ssrc);
            fflush(stdout);

            agg_recv = agg_fwd = agg_dup = agg_gap = agg_late = 0;
            t_last   = now;
        }
    }
}
