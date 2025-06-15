#!/bin/sh

IFACE="$1"
MASTER_URL="http://192.168.2.20:8000/channel.txt"

if [ -z "$IFACE" ]; then
    echo "Usage: $0 <interface>"
    exit 1
fi

# Validate width string
is_valid_width() {
    case "$1" in
        HT20|HT40+|HT40-|NOHT|5MHz|10MHz|80MHz|160MHz) return 0 ;;
        *) return 1 ;;
    esac
}

# Read initial channel
CUR_CHAN=$(iw dev "$IFACE" info 2>/dev/null | awk '/channel/ {print $2}' | head -n1)
[ -n "$CUR_CHAN" ] || CUR_CHAN="unknown"

# Main polling loop
while true; do
    LAST_LINE=$(wget -qO- "$MASTER_URL")

    if [ -n "$LAST_LINE" ]; then
        CHAN=$(echo "$LAST_LINE" | sed -n 's/.*channel=\([0-9]*\).*/\1/p')
        WIDTH=$(echo "$LAST_LINE" | sed -n 's/.*width=\([A-Za-z0-9+-]*\).*/\1/p')
        REGION=$(echo "$LAST_LINE" | sed -n 's/.*region=\([A-Z]*\).*/\1/p')

        if ! is_valid_width "$WIDTH"; then
            echo "[$IFACE] Skipping invalid width: $WIDTH"
            sleep 1
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
