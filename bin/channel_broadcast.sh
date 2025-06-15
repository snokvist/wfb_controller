#!/bin/sh

INTERFACE="${1:-service2}"
PORT=5005
BCAST_ADDR="255.255.255.255"
INTERVAL=1

while true; do
    # Parse all from a single call
    LINE=$(iw dev | awk -v iface="$INTERFACE" '
        $1 == "Interface" { found = ($2 == iface) }
        found && /channel/ && ch == "" { ch = $2 }
        found && /width:/ && width == "" { width = $2 }
        END { if (ch && width) print ch, width }
    ')

    [ -n "$LINE" ] || continue

    CHAN=$(echo "$LINE" | awk '{print $1}')
    WIDTH=$(echo "$LINE" | awk '{print $2}')

    case "$WIDTH" in
        20*) WIDTH="HT20" ;;
        40*) WIDTH="HT40" ;;
        80*) WIDTH="HT80" ;;
        160*) WIDTH="HT160" ;;
        *) WIDTH="HT20" ;;
    esac

    REGION=$(iw reg get | awk '/country/ {print $2}' | cut -d: -f1)
    [ -n "$REGION" ] || REGION="US"

    MSG="channel=$CHAN;width=$WIDTH;region=$REGION"
    echo "[MASTER] Broadcasting: $MSG"

    # This must be run in a subshell to avoid blocking
    (echo "$MSG" | nc -u -b "$BCAST_ADDR" "$PORT") &

    sleep "$INTERVAL"
done
