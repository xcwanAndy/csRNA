CC=gcc
CFLAGS=-c -Wall
LDFLAGS=-static
COMMON_SOURCES=common.c

all: loop client server

loop: loop.c
	$(CC) $(LDFLAGS) loop.c -o ../bin/$@

client: client.c common.h
	$(CC) $(LDFLAGS) $< $(COMMON_SOURCES) -o ../bin/$@

server: server.c common.h
	$(CC) $(LDFLAGS) $< $(COMMON_SOURCES) -o ../bin/$@

clean:
	rm ../bin/loop ../bin/client ../bin/server
