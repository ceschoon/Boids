
LIBBOIDSDIR = ../libBoids

CC = g++
CFLAGS = -O3

boids : boids.cpp
	$(CC) $(CFLAGS) -o $@ $^ \
	-lsfml-graphics -lsfml-window -lsfml-system \
	-I$(LIBBOIDSDIR) -L$(LIBBOIDSDIR) -lboids

clean:
	rm boids