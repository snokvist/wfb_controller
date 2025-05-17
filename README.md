./displayer.py --host 192.168.2.20 --ui index.html --stream-cmd "stream video_agg_rx stdout"

./manager.py --config config.ini --key-path private.key --debug


gcc -O2 -o udp_relay_manager udp_relay_manager.c -pthread

./udp_relay_manager --bind 5801:5802 --bind 5803

echo "set 5801 127.0.0.1 6001" | nc 127.0.0.1 9000

echo "status" | nc 127.0.0.1 9000




sudo apt update

sudo apt install build-essential libboost-all-dev

g++ -std=c++17 -O2 -pthread thread_manager.cpp            \
    -lboost_system -lboost_thread -lboost_filesystem      \
    -lboost_program_options                               \
    -o thread_manager
