////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                        rendering methods                               //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



#ifndef RENDERING_H
#define RENDERING_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

using namespace std;

class Boid;
class Wall;
class Ray;


void renderBoidsAsTriangles(sf::RenderWindow &window, vector<Boid> boids,
                            double scaleX, double scaleY);

void renderBoidsHighlight(sf::RenderWindow &window, vector<Boid> boids, 
                          double scaleX, double scaleY, int indexToHighlight);

void renderBoidsAsPoints(sf::RenderWindow &window, vector<Boid> boids, 
                         double scaleX, double scaleY);

void renderForces(sf::RenderWindow &window, vector<Boid> boids, 
                  double scaleX, double scaleY);

void renderWalls(sf::RenderWindow &window, vector<Wall> walls, 
                 double scaleX, double scaleY);

void renderMouse(sf::RenderWindow &window, double mouseX, double mouseY, 
                 double scaleX, double scaleY);


#endif // RENDERING_H
