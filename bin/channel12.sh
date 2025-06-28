#!/bin/sh

if [ "$1" -ge 980 ] && [ "$1" -le 1999 ]; then
    echo "exec gs_toggle_recording" | nc 10.5.0.1 9500
    tx_manager.sh set_tx_power 4

elif [ "$1" -ge 1200 ] && [ "$1" -le 1499 ]; then
    auto_bitrate.sh 3400
    wfb_tx_cmd 8000 set_fec -k 3 -n 4
    tx_manager.sh set_tx_power 4 

elif [ "$1" -ge 1500 ] && [ "$1" -le 2020 ]; then
    auto_bitrate.sh 9800
    wfb_tx_cmd 8000 set_fec -k 7 -n 10
    tx_manager.sh set_tx_power 4
fi

sleep 2
echo "ELRS-XLINK: CPU:&C &B temp:&T &L31 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
