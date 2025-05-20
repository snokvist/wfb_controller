./displayer.py --host 192.168.2.20 --ui index.html --stream-cmd "stream video_agg_rx stdout"

./manager.py --config config.ini --key-path private.key --debug


gcc -O2 -o udp_relay_manager udp_relay_manager.c -pthread

./udp_relay_manager --bind 5801:5802 --bind 5803

echo "set 5801 5802" | nc 127.0.0.1 9000

echo "status" | nc 127.0.0.1 9000


./ant_selector.py --tx-antennas 7f000001000000,7f000001000001 --switch-cmd "printf 'exec select_{adapter}' | nc -N 127.0.0.1 9500"



Link_domain generation:
printf "%d\n" $((0x$(echo -n default | sha1sum | cut -c1-6)))

hash_link_domain(){ printf "%d\n" $((0x$(printf "%s" "$1" | sha1sum | cut -c1-6))); }

def hash_link_domain(link_domain):
    return int.from_bytes(hashlib.sha1(link_domain.encode('utf-8')).digest()[:3], 'big')
