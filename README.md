# Boids

## Summary

This is a little suite of code to make boid simulations. The core of the
implementation is packed in a small library (lib/ folder). Applications
can then be contructed by calling the library's objects (e.g. the basic
boid simulation in the simulation/ folder).

![alt text](gallery/boids_simulation.png?raw=true "Boid Simulation")


## Installation

This code has been developed under Ubuntu 18.04 with the following dependencies:

> g++ 7.5.0 <br>
> libsfml-dev 2.4.2 <br>
> make 4.1 <br>

You must first compile the boids library in the lib/ folder with the make command. 
The desired application can then be build again with the make command:

> cd lib/ <br>
> make <br>
> cd ../simulation <br>
> make <br>

