#!/bin/sh

# Main ranges (±25)
if [ "$1" -ge 975 ] && [ "$1" -le 1025 ]; then
    echo "exec gs_toggle_recording" | nc 10.5.0.1 9500
    tx_manager.sh set_tx_power 0

elif [ "$1" -ge 1175 ] && [ "$1" -le 1225 ]; then
    auto_bitrate.sh 3400
    wfb_tx_cmd 8000 set_fec -k 3 -n 4
    tx_manager.sh set_tx_power 4 

elif [ "$1" -ge 1375 ] && [ "$1" -le 1425 ]; then
    auto_bitrate.sh 4000
    wfb_tx_cmd 8000 set_fec -k 3 -n 4
    tx_manager.sh set_tx_power 3

elif [ "$1" -ge 1575 ] && [ "$1" -le 1625 ]; then
    auto_bitrate.sh 6500
    wfb_tx_cmd 8000 set_fec -k 6 -n 8
    tx_manager.sh set_tx_power 3

elif [ "$1" -ge 1775 ] && [ "$1" -le 1825 ]; then
    auto_bitrate.sh 9800
    wfb_tx_cmd 8000 set_fec -k 7 -n 10
    tx_manager.sh set_tx_power 3

elif [ "$1" -ge 1975 ] && [ "$1" -le 2025 ]; then
    echo "exec gs_toggle_recording" | nc 10.5.0.1 9500

# Offset ranges (target–75 ±25), skipping 1000 since it's unchanged
elif [ "$1" -ge 1100 ] && [ "$1" -le 1150 ]; then
    set_channel.sh 104

elif [ "$1" -ge 1300 ] && [ "$1" -le 1350 ]; then
    set_channel.sh 124

elif [ "$1" -ge 1500 ] && [ "$1" -le 1550 ]; then
    set_channel.sh 140

elif [ "$1" -ge 1700 ] && [ "$1" -le 1750 ]; then
    set_channel.sh 149

elif [ "$1" -ge 1900 ] && [ "$1" -le 1950 ]; then
    set_channel.sh 157
fi

sleep 2
echo "CPU:&C &B temp:&T &L31 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
