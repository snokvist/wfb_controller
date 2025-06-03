#!/bin/sh

if [ "$1" -ge 950 ]  && [ "$1" -le 1050 ]; then
    echo "exec gs_toggle_recording" | nc 10.5.0.1 9500
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1150 ] && [ "$1" -le 1300 ]; then
    auto_bitrate.sh 3400
    wfb_tx_cmd 8000 set_fec -k 3 -n 4
    tx_manager.sh set_tx_power 4 
elif [ "$1" -ge 1350 ] && [ "$1" -le 1450 ]; then
    auto_bitrate.sh 4000
    wfb_tx_cmd 8000 set_fec -k 3 -n 4
    tx_manager.sh set_tx_power 3
elif [ "$1" -ge 1550 ] && [ "$1" -le 1650 ]; then
    auto_bitrate.sh 6500
    wfb_tx_cmd 8000 set_fec -k 6 -n 8
    tx_manager.sh set_tx_power 3
elif [ "$1" -ge 1700 ] && [ "$1" -le 1850 ]; then
    auto_bitrate.sh 9800
    wfb_tx_cmd 8000 set_fec -k 7 -n 10
    tx_manager.sh set_tx_power 3
elif [ "$1" -ge 1950 ] && [ "$1" -le 2050 ]; then
    echo "exec gs_toggle_recording" | nc 10.5.0.1 9500                         
fi

sleep 2
echo "CPU:&C &B temp:&T &L31 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
**
