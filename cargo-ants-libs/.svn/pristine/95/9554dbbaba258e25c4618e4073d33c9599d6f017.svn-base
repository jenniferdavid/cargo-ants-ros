CC = gcc
#CFLAGS = -Wall -O2 -pipe
CFLAGS = -Wall -O0 -g -pipe

HDRS= pqueue.h grid.h estar.h cell.h
SRCS= pqueue.c grid.c estar.c cell.c
OBJS= $(SRCS:.c=.o)

all: gestar test-pqueue test-drag

gestar: gestar.c $(OBJS) Makefile
	$(CC) $(CFLAGS) -o gestar gestar.c `pkg-config --cflags gtk+-2.0` `pkg-config --libs gtk+-2.0` $(OBJS)

test-drag: test-drag.c Makefile
	$(CC) $(CFLAGS) -o test-drag test-drag.c `pkg-config --cflags gtk+-2.0` `pkg-config --libs gtk+-2.0`

test-pqueue: test-pqueue.c $(OBJS) Makefile
	$(CC) $(CFLAGS) -o test-pqueue test-pqueue.c $(OBJS)

pqueue.o: $(HDRS) pqueue.c Makefile
grid.o: $(HDRS) grid.c Makefile
estar.o: $(HDRS) estar.c Makefile

clean:
	rm -rf *~ *.o *.dSYM gestar test-drag test-pqueue
