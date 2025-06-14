#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#define SOCK_PATH "/tmp/log_input.sock"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <app_name>\n", argv[0]);
        return 1;
    }

    char tag[128];
    snprintf(tag, sizeof(tag), "%s", argv[1]);

    int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, SOCK_PATH);

    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("connect");
        return 1;
    }

    // Send initial tag
    send(sockfd, tag, strlen(tag), 0);

    // Read from stdin and send line-by-line
    char input[1024];
    while (fgets(input, sizeof(input), stdin)) {
        char line[1200];
        size_t len = strlen(input);

        // Strip trailing newline
        if (len > 0 && input[len - 1] == '\n')
            input[len - 1] = '\0';

        snprintf(line, sizeof(line), "%s:%s\n", tag, input);
        send(sockfd, line, strlen(line), 0);
    }

    close(sockfd);
    return 0;
}
