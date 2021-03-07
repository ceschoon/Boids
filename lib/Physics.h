////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                   the physics of the simulation                        //
//             (obstacles, forces, time integration,...)                  //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include <random>

#include <SFML/Graphics.hpp>

using namespace std;


class Boid;
class Wall;
class Ray;


// Profiling data
struct ProfilingData
{
	int calls_render;
	int calls_step;
	int calls_allforces;
	int calls_updateNeighbours;
	int calls_collideWalls;
	
	double avg_time_render;
	double avg_time_step;
	double avg_time_allforces;
	double avg_time_updateNeighbours;
	double avg_time_collideWalls;
};


extern ProfilingData profData_;


/////////////////////////////// World class ////////////////////////////////


class World
{
	public:
		
		// Object Construction
		World(double sizeX, double sizeY, int seed=0);
		
		// Accessors
		int getSeed() {return seed_;}
		int getNumBoids() {return boids_.size();}
		Boid getBoid(int i=0);
		vector<Boid> getBoids();
		
		// Rendering
		void render(sf::RenderWindow &window);
		void renderDebug(sf::RenderWindow &window, int i, bool doForces=false);
		
		// Initialisation
		virtual void placeBoids(vector<Boid*> boids);
		virtual void addRandomWall();
		void addRandomWallAnywhere();
		void addRandomWallOnSquareGrid();
		
		// Time Integration
		void advanceTime(double T, double dt);
		void stepRaw(double dt);
		void collideWalls(const vector<Boid> &boidsOld);
		
		// Profiling
		void printProfilingData();
	
	protected:
		
		double sizeX_;
		double sizeY_;
		
		vector<Wall> walls_;
		vector<Boid*> boids_;
		
		int seed_;
		default_random_engine gen_;
};


////////////////////////// Wall and Ray classes ////////////////////////////


class Wall
{
	public:
		
		Wall(double x1_, double y1_, double x2_, double y2_, bool isABorder_=false)
		{
			x1 = x1_;
			y1 = y1_;
			x2 = x2_;
			y2 = y2_;
			
			isABorder = isABorder_;
		}
		
		void findClosestPoint(double x, double y, double &xw, double &yw) const;
		
		double x1;
		double y1;
		double x2;
		double y2;
		
		bool isABorder;
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


void intersection(const Ray &ray, const Wall &wall, double &xInt, double &yInt, bool &exists);


////////////////////////////////////////////////////////////////////////////


#endif // PHYSICS_H
