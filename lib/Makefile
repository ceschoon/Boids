
# Makefile for the boids library
# Author: Cedric Schoonen
#
# There are dependencies from SFML that must be linked with this library:
# -lsfml-graphics -lsfml-window -lsfml-system

CC = g++
CFLAGS = -O3 # -fPIC flag needed if building a shared library
             # compile with -fopenmp only if you expect to use many cores

#libboids.so : NeuralNetwork.o Math.o Boid.o Physics.o Rendering.o
#	$(CC) -shared $^ -o $@

libboids.a : NeuralNetwork.o Math.o Boid.o Physics.o Rendering.o
	ar rcs $@ $^
	rm *.o 

NeuralNetwork.o :
	$(CC) $(CFLAGS) -c NeuralNetwork.cpp

Math.o :
	$(CC) $(CFLAGS) -c Math.cpp

Boid.o :
	$(CC) $(CFLAGS) -c Boid.cpp

Physics.o :
	$(CC) $(CFLAGS) -c Physics.cpp

Rendering.o :
	$(CC) $(CFLAGS) -c Rendering.cpp

clean :
	rm *.a
	rm *.o

