/*
 * UDP Relay Manager (C version)
 * Listens on multiple UDP ports, relays to configurable destinations.
 * Control via TCP socket (default port 9000):
 *   set <SRC_PORT> <DST_IP> <DST_PORT>
 *   status
 *
 * Compile with: gcc -O2 -o udp_relay_manager udp_relay_manager.c
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <pthread.h>

#define MAX_RELAYS 32
#define CONTROL_PORT 9000
#define BUFSZ 2048

struct relay {
    int src_port;
    int sock_fd;
    struct sockaddr_in dst_addr;
    int has_dst;
};

static struct relay relays[MAX_RELAYS];
static int relay_count = 0;
static pthread_mutex_t relay_lock = PTHREAD_MUTEX_INITIALIZER;

void *relay_thread(void *arg) {
    struct relay *r = (struct relay *)arg;
    char buf[BUFSZ];
    struct sockaddr_in src_addr;
    socklen_t addrlen = sizeof(src_addr);

    while (1) {
        ssize_t len = recvfrom(r->sock_fd, buf, BUFSZ, 0, (struct sockaddr *)&src_addr, &addrlen);
        if (len > 0) {
            pthread_mutex_lock(&relay_lock);
            if (r->has_dst) {
                sendto(r->sock_fd, buf, len, 0,
                       (struct sockaddr *)&r->dst_addr, sizeof(r->dst_addr));
            }
            pthread_mutex_unlock(&relay_lock);
        }
    }
    return NULL;
}

void *control_thread(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = htons(CONTROL_PORT)
    };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    listen(sock, 4);

    char buf[256];
    while (1) {
        int client = accept(sock, NULL, NULL);
        ssize_t len = read(client, buf, sizeof(buf) - 1);
        if (len > 0) {
            buf[len] = 0;
            if (strncmp(buf, "set", 3) == 0) {
                int port, dst_port;
                char ip[64];
                if (sscanf(buf, "set %d %63s %d", &port, ip, &dst_port) == 3) {
                    pthread_mutex_lock(&relay_lock);
                    for (int i = 0; i < relay_count; i++) {
                        if (relays[i].src_port == port) {
                            relays[i].dst_addr.sin_family = AF_INET;
                            relays[i].dst_addr.sin_port = htons(dst_port);
                            inet_pton(AF_INET, ip, &relays[i].dst_addr.sin_addr);
                            relays[i].has_dst = 1;
                            dprintf(client, "OK\n");
                            goto done;
                        }
                    }
                    dprintf(client, "No such port bound\n");
                    done:
                    pthread_mutex_unlock(&relay_lock);
                } else {
                    dprintf(client, "Usage: set <SRC_PORT> <DST_IP> <DST_PORT>\n");
                }
            } else if (strncmp(buf, "status", 6) == 0) {
                pthread_mutex_lock(&relay_lock);
                for (int i = 0; i < relay_count; i++) {
                    dprintf(client, "Port %d -> %s:%d [%s]\n",
                        relays[i].src_port,
                        relays[i].has_dst ? inet_ntoa(relays[i].dst_addr.sin_addr) : "0.0.0.0",
                        ntohs(relays[i].dst_addr.sin_port),
                        relays[i].has_dst ? "active" : "inactive");
                }
                pthread_mutex_unlock(&relay_lock);
            } else {
                dprintf(client, "Unknown command\n");
            }
        }
        close(client);
    }
    return NULL;
}

int bind_udp_port(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port = htons(port)
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }
    return sock;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s --bind <port> [--bind <port> ...]\n", argv[0]);
        return 1;
    }

    signal(SIGPIPE, SIG_IGN);

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--bind") == 0 && i + 1 < argc) {
            int port = atoi(argv[++i]);
            if (relay_count >= MAX_RELAYS) {
                fprintf(stderr, "Too many bound ports\n");
                break;
            }
            int sock = bind_udp_port(port);
            if (sock >= 0) {
                relays[relay_count].src_port = port;
                relays[relay_count].sock_fd = sock;
                relays[relay_count].has_dst = 0;
                pthread_t tid;
                pthread_create(&tid, NULL, relay_thread, &relays[relay_count]);
                pthread_detach(tid);
                relay_count++;
                printf("Bound UDP %d\n", port);
            }
        }
    }

    pthread_t ctl;
    pthread_create(&ctl, NULL, control_thread, NULL);
    pthread_join(ctl, NULL);
    return 0;
}
