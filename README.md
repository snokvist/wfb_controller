./displayer.py --host 192.168.2.20 --ui index.html --stream-cmd "stream video_agg_rx stdout"

./manager.py --config config.ini --key-path private.key --debug


gcc -O2 -o udp_relay_manager udp_relay_manager.c -pthread

./udp_relay_manager --bind 5801:5802 --bind 5803

echo "set 5801 127.0.0.1 6001" | nc 127.0.0.1 9000

echo "status" | nc 127.0.0.1 9000


./ant_selector.py --tx-antennas 7f000001000000,7f000001000001 --switch-cmd "printf 'exec select_{adapter}' | nc -N 127.0.0.1 9500"
