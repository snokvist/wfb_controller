#!/bin/sh

# Usage:
# ./udp_redirect.sh redirect 5801:5802 5803:5804 ...
# ./udp_redirect.sh clean 5801 5803 ...

TABLE="nat"
CHAIN="OUTPUT"

check_iptables() {
    command -v iptables >/dev/null 2>&1 || {
        echo "iptables not found"; exit 1
    }
}

get_rule_line() {
    SRC_PORT="$1"
    iptables -t "$TABLE" -n --line-numbers -L "$CHAIN" | \
        awk -v port="$SRC_PORT" '$0 ~ "udp dpt:" port {print $1; exit}'
}

redirect_port() {
    SRC_PORT="$1"
    DST_PORT="$2"

    LINE=$(get_rule_line "$SRC_PORT")

    if [ -n "$LINE" ]; then
        echo "Replacing redirect: $SRC_PORT → $DST_PORT"
        iptables -t "$TABLE" -R "$CHAIN" "$LINE" -p udp -d 127.0.0.1 --dport "$SRC_PORT" -j REDIRECT --to-ports "$DST_PORT"
    else
        echo "Adding redirect: $SRC_PORT → $DST_PORT"
        iptables -t "$TABLE" -A "$CHAIN" -p udp -d 127.0.0.1 --dport "$SRC_PORT" -j REDIRECT --to-ports "$DST_PORT"
    fi
}

clean_port() {
    PORT="$1"
    while true; do
        LINE=$(get_rule_line "$PORT")
        if [ -n "$LINE" ]; then
            echo "Removing redirect for port $PORT (line $LINE)"
            iptables -t "$TABLE" -D "$CHAIN" "$LINE"
        else
            break
        fi
    done
}

main() {
    check_iptables

    case "$1" in
        redirect)
            shift
            [ $# -ge 1 ] || { echo "Usage: $0 redirect <src:dst> [src:dst]..."; exit 1; }

            for pair in "$@"; do
                SRC=$(echo "$pair" | cut -d: -f1)
                DST=$(echo "$pair" | cut -d: -f2)
                if [ -z "$SRC" ] || [ -z "$DST" ]; then
                    echo "Invalid pair: $pair (expected format: src:dst)"
                    continue
                fi
                redirect_port "$SRC" "$DST"
            done
            ;;
        clean)
            shift
            [ $# -ge 1 ] || { echo "Usage: $0 clean <port> [port]..."; exit 1; }

            for port in "$@"; do
                clean_port "$port"
            done
            ;;
        *)
            echo "Usage:"
            echo "  $0 redirect <src:dst> [src:dst]..."
            echo "  $0 clean <port> [port]..."
            exit 1
            ;;
    esac
}

main "$@"
