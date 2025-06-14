# Makefile for hybrid RTSP server with RTP demux + stats

CC = gcc
CFLAGS = -Wall -O2 $(shell pkg-config --cflags gstreamer-1.0 gstreamer-base-1.0 gstreamer-rtsp-server-1.0)
LIBS = $(shell pkg-config --libs gstreamer-1.0 gstreamer-base-1.0 gstreamer-rtsp-server-1.0)

TARGET = rtsp_hybrid
SRC = main.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f $(TARGET)

# Run example:
# ./rtsp_hybrid <port> <latency> <h264|h265>
