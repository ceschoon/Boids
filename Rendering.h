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

#include "Math.h"
#include "Boid.h"
#include "Physics.h"

using namespace std;


void renderBoidsAsTriangles(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::CircleShape triangle(1,3);
    triangle.setScale(0.5*scaleX*0.3,1*scaleY*0.3);
    triangle.setOrigin(1,1);

    for (int i=0; i<boids.size(); i++)
    {
        triangle.setFillColor(sf::Color::Blue);
        triangle.setPosition(sf::Vector2f(boids[i].x*scaleX,boids[i].y*scaleY));
        triangle.setRotation(boids[i].orientation()+90);
        window.draw(triangle);
    }
}

void renderBoidsHighlight(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY, int indexToHighlight)
{
    sf::CircleShape triangle(1,3);
    triangle.setScale(0.5*scaleX*0.3,1*scaleY*0.3);
    triangle.setOrigin(1,1);

    for (int i=0; i<boids.size(); i++)
    {
    	if (i==indexToHighlight) triangle.setFillColor(sf::Color::Red);
    	else if (isNeighbour(i,boids[indexToHighlight])) triangle.setFillColor(sf::Color::Green);
        else triangle.setFillColor(sf::Color::Blue);
        
        triangle.setPosition(sf::Vector2f(boids[i].x*scaleX,boids[i].y*scaleY));
        triangle.setRotation(boids[i].orientation()+90);
        window.draw(triangle);
    }
}

void renderBoidsAsPoints(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::CircleShape circle(0.1);
    circle.setScale(scaleX,scaleY);
    circle.setOrigin(0.1,0.1);
    circle.setFillColor(sf::Color::Blue);

    for (int i=0; i<boids.size(); i++)
    {
        circle.setPosition(sf::Vector2f(
        	boids[i].x*scaleX,
        	boids[i].y*scaleY
        	));
        window.draw(circle);
    }
}

void renderForces(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::RectangleShape line(sf::Vector2f(1, 0.03));
    line.setFillColor(sf::Color::Black);

    for (int i=0; i<boids.size(); i++)
    {
    	double F = sqrt(boids[i].fx*boids[i].fx+boids[i].fy*boids[i].fy);
    	double angleF = angle(0,0,boids[i].fx,boids[i].fy);
    	line.setScale(scaleX*F/4,scaleY);
        line.setPosition(sf::Vector2f(
        	boids[i].x*scaleX,
        	boids[i].y*scaleY
        	));
        line.setRotation(angleF);
        window.draw(line);
    }
}

void renderWalls(sf::RenderWindow &window, vector<Wall> walls, double scaleX, double scaleY)
{
    sf::RectangleShape line(sf::Vector2f(1, 0.05));
	line.setOrigin(0,0.025);
    line.setFillColor(sf::Color::Black);

    for (int i=0; i<walls.size(); i++)
    {
    	double x1 = walls[i].x1;
    	double y1 = walls[i].y1;
    	double x2 = walls[i].x2;
    	double y2 = walls[i].y2;
    	
    	double L = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    	double angleL = angle(x1,y1,x2,y2);
    	line.setScale(scaleX*L,scaleY);
        line.setPosition(sf::Vector2f(x1*scaleX, y1*scaleY));
        line.setRotation(angleL);
        window.draw(line);
    }
}

void renderMouse(sf::RenderWindow &window, double mouseX, double mouseY, double scaleX, double scaleY)
{
    sf::CircleShape circle(0.1);
    circle.setScale(scaleX,scaleY);
    circle.setOrigin(0.1,0.1);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(sf::Vector2f(mouseX*scaleX, mouseY*scaleY));
    window.draw(circle);
}


#endif // RENDERING_H
