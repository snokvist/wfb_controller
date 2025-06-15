#!/bin/bash

_cleanup() {
  local pids
  pids=$(jobs -pr) && [[ -n $pids ]] && kill -TERM $pids 2>/dev/null || true
}
trap _cleanup EXIT


log_collector --quiet &
sleep 1
wfb-service.sh | log_writer wfb_service &
sleep 1
webui.py --web-port 5010 --ui /usr/bin/index2.html --stream-cmd "wfb_rx,mavlink_rx" --host 127.0.0.1 2>&1| log_writer webui &
mavlink_rx -d /dev/ttyS3 --command-parse --channels 5,8 --min-change 20 --period 1000 2>&1| log_writer mavlink_rx &
wfb_rx -a 10000 -p 0 -U /run/wfb_video.sock -K /etc/openipc.key -R 2097152 -s 2097152 -l 100 -i 7669206 2>&1| log_writer wfb_rx &
xlink.py --verbose | log_writer xlink &
wfb_tx -d -f rts -p 160 -u 5802 -K /etc/openipc.key -B 20 -G long -S 1 -L 1 -M 2 -k 2 -n 5 -T 0 -F 0 -i 7669206 -R 2097152 -s 2097152 -l 1000 -C 8001 127.0.0.1:11004 | log_writer tun_tx &
wfb_rx -a 10002 -p 32 -u 5800 -K /etc/openipc.key -R 2097152 -s 2097152 -l 1000 -i 7669206 | log_writer tun_rx&
udp_relay_manager --bind 5801:5802 --bind 5901:5902 --bind 14451:14452 | log_writer udp_relay &
wfb_tun -t wfb-tun -a 10.5.0.1/24 -u 5801 -l 5800 -T 0 &

wait -n
