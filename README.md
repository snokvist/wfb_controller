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



''
piS relay setup
wlan0 listens on standard channel with standard link domain 7669206
wlan0 retransmits on new relay channel with relay link domain 12396722
adapter on gs listening on channel 104 picks up the relayed data and sends to aggregator
''
#Listen to existing stream
''
wfb_rx -f -c 127.0.0.1 -u 5600 -p 0 -i 7669206 -R 2097152 wlan0 &
''

#retransmit on another channel/adapter
''
wfb_tx -K /etc/openipc.key -M 5 -B 20 -k 8 -n 12 -u 5600 -S 1 -L 1 -i 12396722 -C 8001 wlan1 &
''
