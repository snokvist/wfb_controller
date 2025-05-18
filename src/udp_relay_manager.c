/*
 * UDP Relay Manager – advanced version with robust parsing and stats
 * ---------------------------------------------------------------
 * • Listen on multiple UDP source ports and mirror packets to many destinations.
 * • Bind syntax:  --bind <SRC>[:<DST_LIST>]
 *      example: --bind 5801:9000,9001,7000-7002,192.168.0.10:7500
 *      - If DST_LIST omitted, relay starts with no destinations until "set" cmd.
 *      - Each DST item can be:
 *         • <port>                 (IP defaults to 127.0.0.1)
 *         • <ip:port>              (explicit IP)
 *         • <start-end>            (range, loopback IP)
 * • Control TCP port (default 9000 or --control-port <p>) accepts commands:
 *      set <SRC_PORT> <DST_LIST>   – overwrite destination list
 *      stats | status              – show per-port / per-dest statistics
 *      help                        – show help
 * • Socket buffer sizes overridable: --rcvbuf <bytes>  --sndbuf <bytes>
 *
 * Build: gcc -O2 -pthread -o udp_relay_manager udp_relay_manager.c
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define MAX_RELAYS 32
#define MAX_DESTS  64
#define BUFSZ      2048
#define DEF_CTLPORT 9000

struct dest {
    struct sockaddr_in addr;
    uint64_t pkts_out;
};

struct relay {
    int src_port;
    int sock_fd;
    struct dest dests[MAX_DESTS];
    int dest_cnt;
    uint64_t pkts_in;
};

static struct relay relays[MAX_RELAYS];
static int relay_cnt = 0;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static int control_port = DEF_CTLPORT;
static int rcvbuf_sz = 0, sndbuf_sz = 0;

/* Trim leading/trailing whitespace */
static char *trim(char *s) {
    char *end;
    while (isspace((unsigned char)*s)) s++;
    if (*s == '\0') return s;
    end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) end--;
    *(end+1) = '\0';
    return s;
}

/* Add a single destination */
static void add_dest(struct relay *r, const char *ip, int port) {
    if (r->dest_cnt >= MAX_DESTS) return;
    struct dest *d = &r->dests[r->dest_cnt++];
    memset(d, 0, sizeof(*d));
    d->addr.sin_family = AF_INET;
    d->addr.sin_port   = htons(port);
    inet_pton(AF_INET, ip, &d->addr.sin_addr);
    d->pkts_out = 0;
}

static void clear_dests(struct relay *r) {
    r->dest_cnt = 0;
}

/* Parse tokens like "ip:port" or "start-end" */
static void parse_dest_token(struct relay *r, const char *tok_in) {
    char buf[64];
    strncpy(buf, tok_in, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';
    char *s = trim(buf);
    char *ip_part = NULL;
    char *port_part = s;
    if (strchr(s, ':')) {
        ip_part = strtok(s, ":");
        port_part = strtok(NULL, "");
    }
    const char *ip = ip_part ? ip_part : "127.0.0.1";

    if (strchr(port_part, '-')) {
        int a = atoi(strtok(port_part, "-"));
        int b = atoi(strtok(NULL, ""));
        if (a > b) { int t = a; a = b; b = t; }
        for (int p = a; p <= b; p++) add_dest(r, ip, p);
    } else {
        add_dest(r, ip, atoi(port_part));
    }
}

static void parse_dest_list(struct relay *r, const char *list) {
    clear_dests(r);
    char *tmp = strdup(list);
    char *tok = strtok(tmp, ",");
    while (tok) {
        parse_dest_token(r, tok);
        tok = strtok(NULL, ",");
    }
    free(tmp);
}

/* Create and bind UDP socket with reuse and buffer options */
static int make_udp_socket(int port) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return -1;
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    if (rcvbuf_sz) setsockopt(s, SOL_SOCKET, SO_RCVBUF, &rcvbuf_sz, sizeof(rcvbuf_sz));
    if (sndbuf_sz) setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf_sz, sizeof(sndbuf_sz));

    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(port);
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) { perror("bind"); close(s); return -1; }
    return s;
}

/* Relay loop: receive and forward to all destinations */
static void *relay_loop(void *arg) {
    struct relay *r = (struct relay*)arg;
    char buf[BUFSZ];
    while (1) {
        ssize_t n = recv(r->sock_fd, buf, BUFSZ, 0);
        if (n <= 0) continue;
        pthread_mutex_lock(&lock);
        r->pkts_in++;
        for (int i = 0; i < r->dest_cnt; i++) {
            sendto(r->sock_fd, buf, n, 0,
                   (struct sockaddr*)&r->dests[i].addr,
                   sizeof(r->dests[i].addr));
            r->dests[i].pkts_out++;
        }
        pthread_mutex_unlock(&lock);
    }
    return NULL;
}

/* Control help message */
static void ctl_help(int c) {
    dprintf(c,
        "Commands:\n"
        " set <SRC_PORT> <DST_LIST>   overwrite destinations\n"
        " stats | status              show statistics\n"
        " help                        this help\n");
}

/* Control stats output */
static void ctl_stats(int c) {
    pthread_mutex_lock(&lock);
    for (int i = 0; i < relay_cnt; i++) {
        struct relay *r = &relays[i];
        dprintf(c, "Port %d in_pkts=%" PRIu64 " dests=%d\n",
                r->src_port, r->pkts_in, r->dest_cnt);
        for (int j = 0; j < r->dest_cnt; j++) {
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET,
                      &r->dests[j].addr.sin_addr,
                      ip, sizeof(ip));
            dprintf(c,
                "  -> %s:%d pkts=%" PRIu64 "\n",
                ip,
                ntohs(r->dests[j].addr.sin_port),
                r->dests[j].pkts_out);
        }
    }
    pthread_mutex_unlock(&lock);
}

/* Control loop: robust command parsing */
static void *control_loop(void *arg) {
    (void)arg;
    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(control_port);
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) { perror("ctl bind"); exit(1); }
    listen(s, 4);

    char buf[512];
    while (1) {
        int c = accept(s, NULL, NULL);
        if (c < 0) continue;
        ssize_t n = read(c, buf, sizeof(buf)-1);
        if (n <= 0) { close(c); continue; }
        buf[n] = '\0';
        char *cmd = strtok(buf, " \t\r\n");
        if (!cmd) { ctl_help(c); close(c); continue; }
        if (strcmp(cmd, "set") == 0) {
            char *sport_str = strtok(NULL, " \t\r\n");
            char *list = strtok(NULL, "\r\n");
            if (!sport_str || !list) {
                dprintf(c, "ERR usage: set <port> <list>\n");
            } else {
                int sport = atoi(sport_str);
                pthread_mutex_lock(&lock);
                int found = 0;
                for (int i = 0; i < relay_cnt; i++) {
                    if (relays[i].src_port == sport) {
                        parse_dest_list(&relays[i], list);
                        relays[i].pkts_in = 0;
                        for (int j = 0; j < relays[i].dest_cnt; j++)
                            relays[i].dests[j].pkts_out = 0;
                        found = 1;
                        break;
                    }
                }
                pthread_mutex_unlock(&lock);
                dprintf(c, found ? "OK\n" : "ERR port not bound\n");
            }
        } else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "stats") == 0) {
            ctl_stats(c);
        } else {
            ctl_help(c);
        }
        close(c);
    }
    return NULL;
}

/* Usage info */
static void print_usage(const char *prog) {
    printf("Usage: %s [options] --bind SRC[:DST_LIST] [--bind ...]\n", prog);
    printf("Options:\n"
           "  --control-port <p>    TCP control port (default %d)\n"
           "  --rcvbuf <bytes>      UDP receive buffer\n"
           "  --sndbuf <bytes>      UDP send buffer\n"
           "  --help                Show this help\n\n"
           , DEF_CTLPORT);
    printf("DST_LIST examples: 9000,9001,7000-7005,192.168.0.10:7500\n\n");
}

int main(int argc, char *argv[]) {
    signal(SIGPIPE, SIG_IGN);
    if (argc == 1) { print_usage(argv[0]); return 0; }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0) { print_usage(argv[0]); return 0; }
        else if (strcmp(argv[i], "--control-port") == 0 && i+1 < argc) {
            control_port = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--rcvbuf") == 0 && i+1 < argc) {
            rcvbuf_sz = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--sndbuf") == 0 && i+1 < argc) {
            sndbuf_sz = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--bind") == 0 && i+1 < argc) {
            if (relay_cnt >= MAX_RELAYS) { fprintf(stderr, "Too many binds\n"); break; }
            char *arg = argv[++i];
            char *sep = strchr(arg, ':');
            int sport = atoi(arg);
            relays[relay_cnt].src_port = sport;
            relays[relay_cnt].dest_cnt = 0;
            relays[relay_cnt].pkts_in = 0;
            relays[relay_cnt].sock_fd = make_udp_socket(sport);
            if (relays[relay_cnt].sock_fd < 0) {
                fprintf(stderr, "Failed bind %d\n", sport);
                continue;
            }
            if (sep) parse_dest_list(&relays[relay_cnt], sep+1);
            pthread_t tid;
            pthread_create(&tid, NULL, relay_loop, &relays[relay_cnt]);
            pthread_detach(tid);
            printf("Bound %d with %d dests\n", sport, relays[relay_cnt].dest_cnt);
            relay_cnt++;
        }
        else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    pthread_t ctl;
    pthread_create(&ctl, NULL, control_loop, NULL);
    pthread_join(ctl, NULL);
    return 0;
}
