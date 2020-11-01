
LIBBOIDSDIR = ../lib

CC = g++
CFLAGS = -O3 -fopenmp

boids : boids.cpp
	$(CC) $(CFLAGS) -o $@ $^ \
	-lsfml-graphics -lsfml-window -lsfml-system \
	-I$(LIBBOIDSDIR) -L$(LIBBOIDSDIR) -lboids

clean:
	rm boids