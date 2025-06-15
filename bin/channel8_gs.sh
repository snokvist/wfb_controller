#!/bin/sh

# === Set A: Main action triggers (±20) ===
if [ "$1" -ge 980 ] && [ "$1" -le 1020 ]; then
    echo "A"

elif [ "$1" -ge 1180 ] && [ "$1" -le 1220 ]; then
    echo "A"

elif [ "$1" -ge 1380 ] && [ "$1" -le 1420 ]; then
    echo "A"

elif [ "$1" -ge 1580 ] && [ "$1" -le 1620 ]; then
    echo "A"

elif [ "$1" -ge 1780 ] && [ "$1" -le 1820 ]; then
    echo "A"

elif [ "$1" -ge 1980 ] && [ "$1" -le 2020 ]; then
    echo "A"

# === Set B: Offset -25 (±20) ===
elif [ "$1" -ge 1055 ] && [ "$1" -le 1095 ]; then
    echo "Set B - 1075"
    set_channel.sh service2 48 HT20

elif [ "$1" -ge 1255 ] && [ "$1" -le 1295 ]; then
    echo "Set B - 1275"
        set_channel.sh service2 104 HT20

elif [ "$1" -ge 1455 ] && [ "$1" -le 1495 ]; then
    echo "Set B - 1475"
        set_channel.sh service2 128 HT20

elif [ "$1" -ge 1655 ] && [ "$1" -le 1695 ]; then
    echo "Set B - 1675"
        set_channel.sh service2 140 HT20

elif [ "$1" -ge 1855 ] && [ "$1" -le 1895 ]; then
    echo "Set B - 1875"
        set_channel.sh service2 157 HT20

# 2000 already handled above, no duplicate here for B

# === Set C: Offset -50 (±20) ===
elif [ "$1" -ge 1130 ] && [ "$1" -le 1170 ]; then
    echo "Set C - 1150"

elif [ "$1" -ge 1330 ] && [ "$1" -le 1370 ]; then
    echo "Set C - 1350"

elif [ "$1" -ge 1530 ] && [ "$1" -le 1570 ]; then
    echo "Set C - 1550"

elif [ "$1" -ge 1730 ] && [ "$1" -le 1770 ]; then
    echo "Set C - 1750"

elif [ "$1" -ge 1930 ] && [ "$1" -le 1970 ]; then
    echo "Set C - 1950"

# 2000 already handled above, no duplicate here for C

fi

sleep 2
echo "CPU:&C &B temp:&T &L31 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
