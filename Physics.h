////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                   the physics of the simulation                        //
//             (obstacles, forces, time integration,...)                  //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


// TODO: put all the physics inside the world class


#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include <random>

#include <SFML/Graphics.hpp>

using namespace std;


class Boid;
class Wall;
class Ray;


/////////////////////////////// World class ////////////////////////////////


class World
{
	public:
		// Object Construction
		World(double sizeX, double sizeY);
		
		// Rendering
		void render(sf::RenderWindow &window);
		
		// Initialisation
		void addRandomWall(vector<Wall> &walls, int boxSizeX, int boxSizeY,
		                   default_random_engine &gen);
		
		void addRandomWallOnSquareGrid(vector<Wall> &walls, int boxSizeX, int boxSizeY,
		                               default_random_engine &gen);
		
		void placeBoids(vector<Boid> &boids, double boxSizeX, double boxSizeY,
		                default_random_engine &gen);
		
		// Time Integration
		void advanceTime(vector<Boid> &boids, double dt);
		void collideWalls(vector<Boid> &boidsOld, vector<Boid> &boidsNew, 
		                  vector<Wall> walls);
	
	protected:
		double sizeX_;
		double sizeY_;
		
		vector<Wall> walls_;
};


////////////////////////// Wall and Ray classes ////////////////////////////


class Wall
{
	public:
		Wall(double x1_, double y1_, double x2_, double y2_)
		{
			x1 = x1_;
			y1 = y1_;
			x2 = x2_;
			y2 = y2_;
		}
		
		double x1;
		double y1;
		double x2;
		double y2;
};


class Ray
{
	public:
		Ray(double originX_, double originY_, double angle_)
		{
			originX = originX_;
			originY = originY_;
			angle = angle_;
		}
		
		double originX;
		double originY;
		double angle;
};


void intersection(Ray ray, Wall wall, double &xInt, double &yInt, bool &exists);


////////////////////////////////////////////////////////////////////////////


#endif // PHYSICS_H
