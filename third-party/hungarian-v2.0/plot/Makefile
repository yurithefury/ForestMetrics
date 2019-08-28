
CC = g++
CFLAGS = -O3 -Wall
LDFLAGS =

#all: Cgnuplot

%.o: %.c %.h
	$(CC) $(CFLAGS) -c $<
Cgnuplot.o:
	$(CC) $(CFLAGS) -c Cgnuplot.c

clean:
	rm -f *.o *~ *.swp Cgnuplot

