#!/bin/sh

INTERFACE="${1:-service2}"
PORT=5005
BCAST_ADDR="255.255.255.255"
INTERVAL=1

while true; do
    LINE=$(iw dev | awk -v iface="$INTERFACE" '
        $1 == "Interface" { in_block = ($2 == iface) }
        in_block && $1 == "channel" {
            print
            exit
        }')

    CHAN=$(echo "$LINE" | awk '{print $2}')
    WIDTH_RAW=$(echo "$LINE" | sed -n 's/.*width: \([0-9]*\) MHz.*/\1/p')
    CENTER1=$(echo "$LINE" | sed -n 's/.*center1: \([0-9]*\) MHz.*/\1/p')

    # Compute primary channel frequency
    FREQ=$((5000 + 5 * CHAN))

    # Detect width
    case "$WIDTH_RAW" in
        20)
            WIDTH="HT20"
            ;;
        40)
            if [ "$CENTER1" -gt "$FREQ" ]; then
                WIDTH="HT40+"
            else
                WIDTH="HT40-"
            fi
            ;;
        80)
            WIDTH="80MHz"
            ;;
        160)
            WIDTH="160MHz"
            ;;
        10)
            WIDTH="10MHz"
            ;;
        5)
            WIDTH="5MHz"
            ;;
        *)
            WIDTH="HT20"
            ;;
    esac

    REGION=$(iw reg get | awk '/country/ {print $2}' | cut -d: -f1)
    [ -n "$REGION" ] || REGION="US"

    MSG="channel=$CHAN;width=$WIDTH;region=$REGION"
    echo "[MASTER] Broadcasting: $MSG"
    (echo "$MSG" | nc -u -b "$BCAST_ADDR" "$PORT") &

    sleep "$INTERVAL"
done
