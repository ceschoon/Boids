#! /bin/bash

g++ -O3 -o NNObstacles \
	main.cpp NeuralNetwork.cpp Math.cpp Boid.cpp Physics.cpp Rendering.cpp \
	-lsfml-graphics -lsfml-window -lsfml-system