#!/bin/sh
echo "Running logic on msposd channel 5"

if [ "$1" -lt 1200 ]; then
    echo "Disarm detected, setting low power"
    #iw wlan0 set txpower fixed 100
    tx_manager.sh set_tx_power 1
elif [ "$1" -gt 1800 ]; then
     echo "ARM detected, setting high power"
     #iw wlan0 set txpower fixed 2750
     tx_manager.sh set_tx_power 4
fi
    ##TX ON ARM END

exit 0
