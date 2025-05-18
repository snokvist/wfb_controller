./displayer.py --host 192.168.2.20 --ui index.html --stream-cmd "stream video_agg_rx stdout"

./manager.py --config config.ini --key-path private.key --debug


gcc -O2 -o udp_relay_manager udp_relay_manager.c -pthread

./udp_relay_manager --bind 5801:5802 --bind 5803

echo "set 5801 127.0.0.1 6001" | nc 127.0.0.1 9000

echo "status" | nc 127.0.0.1 9000


g++ -std=c++17 -O2 -pthread thread_manager.cpp \
    -lboost_system -lboost_thread -lboost_filesystem \
    -lboost_program_options -o thread_manager

sudo apt install build-essential \
    libboost-system-dev libboost-filesystem-dev \
    libboost-thread-dev libboost-program-options-dev



''
    | Where you build                                                 | Recommended compiler      | Packages to install (Debian/Ubuntu examples)                                                                                                             | Typical command                                                                                                                                                                                           |
| --------------------------------------------------------------- | ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Native armhf board** (Raspberry Pi 3, Radxa Zero 1…)          | `g++` 10+                 | `sudo apt install build-essential libboost-all-dev`                                                                                                      | `g++ -std=c++17 -O2 -pthread thread_manager.cpp $(pkg-config --libs --cflags boost) -o thread_manager`                                                                                                    |
| **Native arm64 board** (Raspberry Pi 4/5, Radxa Zero 3, Jetson) | Same as above             | Same                                                                                                                                                     | Same                                                                                                                                                                                                      |
| **Native x86-64 PC / VM**                                       | Same                      | Same                                                                                                                                                     | Same                                                                                                                                                                                                      |
| **Cross-compile on x86-64 for armhf**                           | `arm-linux-gnueabihf-g++` | `sudo apt install g++-arm-linux-gnueabihf libboost-all-dev:armhf`<br>(and enable `armhf` as a foreign arch: `dpkg --add-architecture armhf; apt update`) | `bash CXX=arm-linux-gnueabihf-g++ \ g++ -std=c++17 -O2 -pthread thread_manager.cpp \ $(pkg-config --libs --cflags --static --define-variable=prefix=/usr/arm-linux-gnueabihf boost) \ -o thread_manager ` |
| **Cross-compile on x86-64 for arm64**                           | `aarch64-linux-gnu-g++`   | `sudo apt install g++-aarch64-linux-gnu libboost-all-dev:arm64` (add `arm64` arch first)                                                                 | Same pattern as above with `aarch64-linux-gnu-g++`                                                                                                                                                        |
''
