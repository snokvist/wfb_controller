#!/bin/sh
#
# Usage:  chan_toggle.sh <pwm-value>
#  950–1050  -> step **down**
# 1450–1550  -> **neutral** (no change)
# 1950–2050  -> step **up**

CHANNELS=" 36 40 44 48 104 124 149 153 157 161"

# ---------------------------------------------------------------------------
# helper: count how many items in $CHANNELS
count_channels() {
    set -- $CHANNELS
    echo $#
}

# helper: locate the index (0-based) of $1 inside $CHANNELS
index_of() {
    needle="$1"
    idx=0
    for c in $CHANNELS; do
        [ "$c" = "$needle" ] && { echo $idx; return; }
        idx=$((idx+1))
    done
    # unknown channel – default to first entry
    echo 0
}

# ---------------------------------------------------------------------------
# 1. discover the *current* channel
current=$(iw dev wlan0 info 2>/dev/null | awk '/channel/ {print $2; exit}')
[ -z "$current" ] && current=104             # fall back if parse fails

total=$(count_channels)
cur_idx=$(index_of "$current")

# 2. work out the direction from $1
pwm="$1"
case "$pwm" in
    ''|*[!0-9]*)  echo "Need a numeric PWM value" >&2; exit 1 ;;
esac

if   [ "$pwm" -ge  950 ] && [ "$pwm" -le 1050 ]; then dir=-1; msg="down"
elif [ "$pwm" -ge 1450 ] && [ "$pwm" -le 1550 ]; then echo "Neutral – keeping channel $current"; exit 0
elif [ "$pwm" -ge 1950 ] && [ "$pwm" -le 2050 ]; then dir=+1; msg="up"
else
    echo "PWM $pwm not inside any recognised band" >&2
    exit 1
fi

# 3. compute the *new* index, wrapping with modulo arithmetic
new_idx=$(( (cur_idx + dir + total) % total ))

# 4. map the new index back to an actual channel number
idx=0
for ch in $CHANNELS; do
    [ $idx -eq $new_idx ] && { new_chan=$ch; break; }
    idx=$((idx+1))
done

# 5. change channel                                                          
echo "Channel $msg: $current -> $new_chan" 
echo "CPU:&C &B temp:&T\nSwitching channel: $current -> $new_chan &L31 &G8 &F30" > /tmp/MSPOSD.msg  
sleep 1.2           
iw dev wlan0 set channel "$new_chan" HT20
 
# 6. OSD/status message
sleep 1.2                                                  
echo "CPU:&C &B temp:&T\nCurrent channel: $new_chan &L31 &G8 &F30" > /tmp/MSPOSD.msg
exit 0 
