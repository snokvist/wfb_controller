#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>

#define SOCK_PATH "/tmp/log_input.sock"

int connect_with_retry(const char *path, const char *tag) {
    int sockfd;
    struct sockaddr_un addr = {0};

    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    while (1) {
        sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("socket");
            sleep(3);
            continue;
        }

        if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
            if (send(sockfd, tag, strlen(tag), 0) < 0) {
                perror("send tag");
                close(sockfd);
                sleep(3);
                continue;
            }
            return sockfd;
        }

        perror("connect");
        close(sockfd);
        sleep(3);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <app_name>\n", argv[0]);
        return 1;
    }

    char tag[128];
    snprintf(tag, sizeof(tag), "%s", argv[1]);

    int sockfd = connect_with_retry(SOCK_PATH, tag);

    char input[1024];
    while (fgets(input, sizeof(input), stdin)) {
        char line[1200];
        size_t len = strlen(input);

        // Strip newline
        if (len > 0 && input[len - 1] == '\n')
            input[len - 1] = '\0';

        snprintf(line, sizeof(line), "%s:%s\n", tag, input);

        ssize_t sent = send(sockfd, line, strlen(line), 0);
        if (sent < 0) {
            perror("send");
            close(sockfd);
            fprintf(stderr, "[INFO] Dropped line and reconnecting...\n");
            sockfd = connect_with_retry(SOCK_PATH, tag);
            // Do not resend the dropped line
        }
    }

    close(sockfd);
    return 0;
}
