./displayer.py --host 192.168.2.20 --ui index.html --stream-cmd "stream video_agg_rx stdout"

./manager.py --config config.ini --key-path private.key --debug


gcc -O2 -o udp_relay_manager udp_relay_manager.c -pthread


echo "set 5801 127.0.0.1 6001" | nc 127.0.0.1 9000

echo "status" | nc 127.0.0.1 9000
