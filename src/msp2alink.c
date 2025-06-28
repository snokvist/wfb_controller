/* msp_to_alink.c  –  MSP-RC → alink UDP bridge
 *
 * Build:
 *   arm-linux-gnueabihf-gcc -O2 -Wall -o msp2alink msp_to_alink.c
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>

/* ---------- defaults ---------- */
#define DEF_LISTEN_ADDR "127.0.0.1"
#define DEF_LISTEN_PORT  9999
#define DEF_DEST_PORT   10000
#define DEF_RSSI_MIN    -100
#define DEF_RSSI_MAX      10
#define MSP_RC_CMD        105          /* 0x69 */
#define MSP_HDR_LEN         3

static const char *listen_addr = DEF_LISTEN_ADDR;
static int  listen_port  = DEF_LISTEN_PORT;
static int  dest_port    = DEF_DEST_PORT;
static int  rate_hz      = 0;          /* 0 = passthrough */
static int  rssi_min     = DEF_RSSI_MIN;
static int  rssi_max     = DEF_RSSI_MAX;
static int  min_value    = 0;          /* 0 = disabled */
static bool debug_flag   = false;

static uint64_t fixed_start_epoch = 1724872943ULL;   /* arbitrary */

/* ---------- helpers ---------- */
static inline uint64_t mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}
static inline uint64_t mono_sec(void) { return mono_us() / 1000000ULL; }

static void usage(const char *p)
{
    fprintf(stderr,
        "Usage: %s [OPTIONS]\n"
        "  --listen-addr=IP      (default %s)\n"
        "  --listen-port=N       (default %d)\n"
        "  --dest-port=N         (default %d)\n"
        "  --rate=N              (Hz, default passthrough)\n"
        "  --min-value=N         (≤N sent as 999, default off)\n"
        "  --rssi-min=N          (default %d)\n"
        "  --rssi-max=N          (default %d)\n"
        "  --debug | -d          (verbose prints)\n"
        "  --help  | -h\n",
        p, DEF_LISTEN_ADDR, DEF_LISTEN_PORT, DEF_DEST_PORT,
        DEF_RSSI_MIN, DEF_RSSI_MAX);
}

static void parse_args(int ac, char **av)
{
    for (int i = 1; i < ac; ++i) {
        if (!strcmp(av[i], "--help") || !strcmp(av[i], "-h"))
            { usage(av[0]); exit(0); }
        else if (!strcmp(av[i], "--debug") || !strcmp(av[i], "-d"))
            debug_flag = true;
        else if (!strncmp(av[i], "--listen-addr=", 14))
            listen_addr = av[i] + 14;
        else if (!strncmp(av[i], "--listen-port=", 14))
            listen_port = atoi(av[i] + 14);
        else if (!strncmp(av[i], "--dest-port=", 12))
            dest_port = atoi(av[i] + 12);
        else if (!strncmp(av[i], "--rate=", 7))
            rate_hz = atoi(av[i] + 7);
        else if (!strncmp(av[i], "--rssi-min=", 11))
            rssi_min = atoi(av[i] + 11);
        else if (!strncmp(av[i], "--rssi-max=", 11))
            rssi_max = atoi(av[i] + 11);
        else if (!strncmp(av[i], "--min-value=", 12))
            min_value = atoi(av[i] + 12);
        else {
            fprintf(stderr, "Unknown option %s\n", av[i]);
            usage(av[0]); exit(1);
        }
    }
}

static int map_rssi(int ch)
{
    if (ch < 1000) ch = 1000;
    if (ch > 2000) ch = 2000;
    int span = rssi_max - rssi_min;
    return rssi_min + (int)((ch - 1000) * span / 1000);
}

/* ---------- main ---------- */
int main(int argc, char **argv)
{
    parse_args(argc, argv);

    if (debug_flag) {
        printf("[CFG] listen %s:%d  → 127.0.0.1:%d  rate=%s%s  rssi=%d..%d  min=%d\n",
               listen_addr, listen_port, dest_port,
               rate_hz ? "" : "pass-", rate_hz ? "fixed" : "through",
               rssi_min, rssi_max, min_value);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    struct sockaddr_in a_listen = {0};
    a_listen.sin_family = AF_INET;
    a_listen.sin_port   = htons(listen_port);
    if (!inet_aton(listen_addr, &a_listen.sin_addr)) {
        fprintf(stderr, "Invalid listen address %s\n", listen_addr);
        return 1;
    }
    if (bind(sock, (struct sockaddr*)&a_listen, sizeof(a_listen)) < 0) {
        perror("bind"); return 1;
    }

    struct sockaddr_in a_send = {0};
    a_send.sin_family      = AF_INET;
    a_send.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a_send.sin_port        = htons(dest_port);

    /* state */
    int      latest_ch  = 1500;
    uint64_t base_us    = mono_us();
    uint64_t last_tx_us = 0;
    uint64_t period_us  = (rate_hz ? 1000000ULL / rate_hz : 0);

    /* buffers */
    unsigned char rx[512];
    char          tx[128];

    while (1) {
        uint64_t now_us;
        bool     do_tx;

        /* --- select() --- */
        fd_set rfds; FD_ZERO(&rfds); FD_SET(sock, &rfds);
        struct timeval tv;
        if (rate_hz) {
            tv.tv_sec  = period_us / 1000000ULL;
            tv.tv_usec = period_us % 1000000ULL;
        } else { tv.tv_sec = 5; tv.tv_usec = 0; }

        int sel = select(sock + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0 && errno == EINTR) continue;
        if (sel < 0) { perror("select"); break; }

        /* --- receive MSP packet --- */
        if (sel > 0 && FD_ISSET(sock, &rfds)) {
            ssize_t len = recv(sock, rx, sizeof(rx), 0);
            if (len >= (MSP_HDR_LEN + 3) &&
                rx[0] == '$' && rx[1] == 'M' &&
               (rx[2] == '<' || rx[2] == '>'))
            {
                uint8_t size = rx[3];
                uint8_t cmd  = rx[4];
                if (cmd == MSP_RC_CMD && size >= 32 &&
                    len >= (ssize_t)(5 + size + 1))
                {
                    uint8_t *p = &rx[5];
                    int16_t ch[16];
                    for (int i = 0; i < 16; ++i)
                        ch[i] = p[i*2] | (p[i*2+1] << 8);
                    latest_ch = ch[11];
                    if (debug_flag) printf("[RX ] ch12=%d\n", latest_ch);
                }
            }
        }

        /* --- decide TX --- */
        now_us = mono_us();
        do_tx  = false;
        if (rate_hz == 0) {
            if (sel > 0) do_tx = true;
        } else {
            if (now_us - last_tx_us >= period_us) do_tx = true;
        }

        if (do_tx) {
            int send_val = latest_ch;
            if (min_value && send_val <= min_value) {
                send_val = 999;
                if (debug_flag) printf("[CLP] value ≤%d → 999\n", min_value);
            }

            int rssi = map_rssi(send_val);

            snprintf(tx, sizeof(tx),
                     "%llu:%d:%d:0:0:%d:0:2:0:0",
                     (unsigned long long)(fixed_start_epoch +
                        (mono_sec() - base_us / 1000000ULL)),
                     send_val, send_val, rssi);

            sendto(sock, tx, strlen(tx), 0,
                   (struct sockaddr*)&a_send, sizeof(a_send));

            if (debug_flag) printf("[TX ] %s\n", tx);

            last_tx_us = now_us;
        }
    }
    return 0;
}
