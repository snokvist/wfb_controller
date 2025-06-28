// All logs
// printf '\n' | nc 127.0.0.1 9500
// Only IDs that start with "auth" or "db"
// printf 'auth,db\n' | nc 127.0.0.1 9500

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#define UNIX_SOCK_PATH "/tmp/log_input.sock"
#define TCP_PORT 9500
#define MAX_LINE 1024
#define MAX_CLIENTS 10
#define MAX_FILTER 10

typedef struct {
    int fd;
    char *filters[MAX_FILTER];
    int filter_count;
} tcp_client_t;

pthread_mutex_t clients_lock = PTHREAD_MUTEX_INITIALIZER;
tcp_client_t tcp_clients[MAX_CLIENTS];
int quiet_mode = 0;

int client_filter_match(tcp_client_t *client, const char *id) {
    if (client->filter_count == 0) return 1;
    for (int i = 0; i < client->filter_count; i++) {
        if (strncmp(id, client->filters[i], strlen(client->filters[i])) == 0)
            return 1;
    }
    return 0;
}

void broadcast_line(const char *line, const char *id) {
    pthread_mutex_lock(&clients_lock);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        tcp_client_t *client = &tcp_clients[i];
        if (client->fd > 0 && client_filter_match(client, id)) {
            if (send(client->fd, line, strlen(line), MSG_NOSIGNAL) < 0) {
                close(client->fd);
                client->fd = -1;
            }
        }
    }
    pthread_mutex_unlock(&clients_lock);
}

void *handle_unix_client(void *arg) {
    int fd = *(int *)arg;
    free(arg);
    char id[128] = "UNKNOWN";
    ssize_t n = recv(fd, id, sizeof(id) - 1, 0);
    if (n <= 0) {
        close(fd);
        return NULL;
    }
    id[n] = '\0';

    if (!quiet_mode) {
        printf("[INFO] UNIX client connected with ID: %s\n", id);
        fflush(stdout);
    }

    char line[MAX_LINE];
    while ((n = recv(fd, line, sizeof(line) - 1, 0)) > 0) {
        line[n] = '\0';

        if (!quiet_mode) {
            fputs(line, stdout);
            if (line[n - 1] != '\n') putchar('\n');
            fflush(stdout);
        }

        broadcast_line(line, id);
    }
    close(fd);
    return NULL;
}

void *unix_server_loop(void *_) {
    int sockfd;
    struct sockaddr_un addr;

    unlink(UNIX_SOCK_PATH);
    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, UNIX_SOCK_PATH);

    bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    listen(sockfd, 5);

    if (!quiet_mode) {
        printf("[INFO] UNIX socket listening: %s\n", UNIX_SOCK_PATH);
    }

    while (1) {
        int *client_fd = malloc(sizeof(int));
        *client_fd = accept(sockfd, NULL, NULL);
        pthread_t t;
        pthread_create(&t, NULL, handle_unix_client, client_fd);
        pthread_detach(t);
    }
}

void parse_filters(tcp_client_t *client, const char *input) {
    client->filter_count = 0;
    char *copy = strdup(input);
    char *tok = strtok(copy, ",");
    while (tok && client->filter_count < MAX_FILTER) {
        if (*tok != '\0') {  // skip empty tokens
            client->filters[client->filter_count++] = strdup(tok);
        }
        tok = strtok(NULL, ",");
    }
    free(copy);
}

void *tcp_server_loop(void *_) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 5) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    if (!quiet_mode) {
        printf("[INFO] TCP subscriber server on port %d\n", TCP_PORT);
    }

    while (1) {
        int client_fd = accept(server_fd, NULL, NULL);
        char buf[256] = {0};
        int n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) {
            close(client_fd);
            continue;
        }

        buf[n] = '\0';
        char *newline = strchr(buf, '\n');
        if (newline) *newline = '\0';  // trim newline

        pthread_mutex_lock(&clients_lock);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (tcp_clients[i].fd <= 0) {
                tcp_clients[i].fd = client_fd;
                parse_filters(&tcp_clients[i], buf);
                if (!quiet_mode) {
                    printf("[INFO] TCP client connected, filters: %s\n", buf);
                    fflush(stdout);
                }
                break;
            }
        }
        pthread_mutex_unlock(&clients_lock);
    }
}


void *keepalive_thread(void *_) {
    while (1) {
        sleep(10);
        pthread_mutex_lock(&clients_lock);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (tcp_clients[i].fd > 0) {
                if (send(tcp_clients[i].fd, "\n", 1, MSG_NOSIGNAL) < 0) {
                    close(tcp_clients[i].fd);
                    tcp_clients[i].fd = -1;
                }
            }
        }
        pthread_mutex_unlock(&clients_lock);
    }
}

int main(int argc, char *argv[]) {
    if (argc > 1 && strcmp(argv[1], "--quiet") == 0) {
        quiet_mode = 1;
    }

    memset(tcp_clients, 0, sizeof(tcp_clients));
    for (int i = 0; i < MAX_CLIENTS; i++) tcp_clients[i].fd = -1;

    pthread_t unix_thread, tcp_thread, keepalive_t;
    pthread_create(&unix_thread, NULL, unix_server_loop, NULL);
    pthread_create(&tcp_thread, NULL, tcp_server_loop, NULL);
    pthread_create(&keepalive_t, NULL, keepalive_thread, NULL);

    pthread_join(unix_thread, NULL);
    return 0;
}
