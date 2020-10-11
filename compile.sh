#! /bin/bash

g++ -O3 -o NNObstacles \
	main.cpp NeuralNetwork.cpp rendering.cpp \
	-lsfml-graphics -lsfml-window -lsfml-system