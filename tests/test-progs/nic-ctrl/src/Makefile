all: loop client server

loop: loop.c
	gcc -static ./loop.c -o ../bin/loop

client: client.c
	gcc -static ./client.c -o ../bin/client

server: server.c
	gcc -static ./server.c -o ../bin/server

clean:
	rm ../bin/loop ../bin/client ../bin/server
