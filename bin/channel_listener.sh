#!/bin/sh

IFACE="$1"
PORT=5005

if [ -z "$IFACE" ]; then
    echo "Usage: $0 <interface>"
    exit 1
fi

SOCKET="/tmp/channel_$IFACE.sock.$$"

cleanup() {
    [ -e "$SOCKET" ] && rm -f "$SOCKET"
}
trap cleanup EXIT

mkfifo "$SOCKET"

CUR_CHAN=$(iw dev "$IFACE" info | awk '/channel/ {print $2}' | head -n1)
[ -n "$CUR_CHAN" ] || CUR_CHAN="unknown"

while true; do
    socat -u UDP-RECV:$PORT STDOUT
done > "$SOCKET" 2>/dev/null &

is_valid_width() {
    case "$1" in
        HT20|HT40+|HT40-|NOHT|5MHz|10MHz|80MHz|160MHz) return 0 ;;
        *) return 1 ;;
    esac
}

while true; do
    LAST_LINE=""
    while read -t 1 LINE < "$SOCKET"; do
        LAST_LINE="$LINE"
    done

    if [ -n "$LAST_LINE" ]; then
        CHAN=$(echo "$LAST_LINE" | sed -n 's/.*channel=\([0-9]*\).*/\1/p')
        WIDTH=$(echo "$LAST_LINE" | sed -n 's/.*width=\([A-Za-z0-9+]*\).*/\1/p')
        REGION=$(echo "$LAST_LINE" | sed -n 's/.*region=\([A-Z]*\).*/\1/p')

        if ! is_valid_width "$WIDTH"; then
            echo "[$IFACE] Skipping invalid width: $WIDTH"
            continue
        fi

        if [ -n "$CHAN" ] && [ "$CHAN" != "$CUR_CHAN" ]; then
            echo "[$IFACE] Attempting switch: $CUR_CHAN → $CHAN width $WIDTH region $REGION"
            iw reg set "$REGION"
            if iw dev "$IFACE" set channel "$CHAN" "$WIDTH"; then
                echo "[$IFACE] Channel switch successful."
                CUR_CHAN="$CHAN"
            else
                echo "[$IFACE] Failed to set channel=$CHAN width=$WIDTH — skipping update."
            fi
        fi
    fi

    sleep 1
done
