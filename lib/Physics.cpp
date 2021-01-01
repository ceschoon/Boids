////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Source file for the implementation of                    //
//                   the physics of the simulation                        //
//             (obstacles, forces, time integration,...)                  //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <omp.h>
#include "Physics.h"
#include "Boid.h"
#include "Math.h"
#include "Rendering.h"

using namespace std;


///////////////////////////////// World ////////////////////////////////////

World::World(double sizeX, double sizeY, int seed)
: sizeX_(sizeX), sizeY_(sizeY)
{
	// Random number generation
	
	if (seed==0)
	{
		random_device true_gen;
		seed = true_gen();
	}
	
	seed_ = seed;
	gen_ = default_random_engine(seed);
	
	// Put walls on the box sides
	
	Wall wallBorder0(0,0,0,sizeY_);
	Wall wallBorder1(0,sizeY_,sizeX_,sizeY_);
	Wall wallBorder2(sizeX_,sizeY_,sizeX_,0);
	Wall wallBorder3(sizeX_,0,0,0);
	
	walls_.push_back(wallBorder0);
	walls_.push_back(wallBorder1);
	walls_.push_back(wallBorder2);
	walls_.push_back(wallBorder3);
}


/////////////////////////////// Accessors //////////////////////////////////

Boid World::getBoid(int i) {return *boids_[i];}

vector<Boid> World::getBoids()
{
	vector<Boid> boids(boids_.size(), Boid(0,0,0,0));
	for (int i=0; i<boids_.size(); i++) boids[i] = *boids_[i];
	
	return boids;
}


//////////////////////// World rendering (call) ////////////////////////////

void World::render(sf::RenderWindow &window)
{
	double scaleX = window.getSize().x/sizeX_;
	double scaleY = window.getSize().y/sizeY_;
	
	renderWalls(window, walls_, scaleX, scaleY);
	renderBoidsAsTriangles(window, getBoids(), scaleX, scaleY);
}


void World::renderDebug(sf::RenderWindow &window, int i, bool doForces)
{
	double scaleX = window.getSize().x/sizeX_;
	double scaleY = window.getSize().y/sizeY_;
	
	renderWalls(window, walls_, scaleX, scaleY);
	renderBoidsHighlight(window, getBoids(), scaleX, scaleY, i);
	if (doForces) renderForces(window, getBoids(), scaleX, scaleY);
}

////////////////////////// Wall - Ray mechanics ////////////////////////////


void intersection(Ray ray, Wall wall, double &xInt, double &yInt, bool &exists)
{
	exists = true;
	
	// this function is called many times, optimise it by
	// storing sin and cos in variables
	double cosAngle = cos(ray.angle*PI/180);
	double sinAngle = sin(ray.angle*PI/180);
	
	double x1 = wall.x1;
	double y1 = wall.y1;
	double x2 = wall.x2;
	double y2 = wall.y2;
	double x3 = ray.originX;
	double y3 = ray.originY;
	double x4 = x3 + cosAngle; // whatever on the ray line
	double y4 = y3 + sinAngle; // whatever on the ray line
	
	if (x1==x2 && x3==x4)
	{
		exists = false;
	}
	else if (x1==x2 && x3!=x4)
	{
		double slope34 = (y4-y3)/(x4-x3);
		
		xInt = x1;
		yInt = y3 + slope34*(xInt-x3);
		
		// check if intersection is on segment 12
		if (yInt<min(y1,y2) || yInt>max(y1,y2)) exists = false;
		
		// check if intersection is in the direction of the ray
		// if ray goes to positive x, intersection must be > x3
		if (cosAngle>0 && xInt<x3) exists = false;
		// if ray goes to negative x, intersection must be < x3
		if (cosAngle<0 && xInt>x3) exists = false;
		// if ray goes to positive y, intersection must be > y3
		if (sinAngle>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sinAngle<0 && yInt>y3) exists = false;
	}
	else if (x1!=x2 && x3==x4)
	{
		double slope12 = (y2-y1)/(x2-x1);
		
		xInt = x3;
		yInt = y1 + 1/slope12*(xInt-x1);
		
		// check if intersection is on segment 12
		if (xInt<min(x1,x2) || xInt>max(x1,x2)) exists = false;
		
		// check if intersection is in the direction of the ray
		// if ray goes to positive y, intersection must be > y3
		if (sinAngle>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sinAngle<0 && yInt>y3) exists = false;
	}
	else
	{
		double slope12 = (y2-y1)/(x2-x1);
		double slope34 = (y4-y3)/(x4-x3);
		
		if (slope12==slope34) exists = false; // parallel
		else
		{
			xInt = (y1-y3-slope12*x1+slope34*x3) / (slope34-slope12);
			yInt = y1 + slope12 * (xInt-x1);
			
			// check if intersection is on segment 12
			if (xInt<min(x1,x2) || xInt>max(x1,x2)) exists = false;
			
			// check if intersection is in the direction of the ray
			// if ray goes to positive x, intersection must be > x3
			if (cosAngle>0 && xInt<x3) exists = false;
			// if ray goes to negative x, intersection must be < x3
			if (cosAngle<0 && xInt>x3) exists = false;
			// if ray goes to positive y, intersection must be > y3
			if (sinAngle>0 && yInt<y3) exists = false;
			// if ray goes to negative y, intersection must be < y3
			if (sinAngle<0 && yInt>y3) exists = false;
		}
	}
}


////////////////////////////// Initialisation //////////////////////////////


void World::addRandomWall()
{
	addRandomWallOnSquareGrid();
}


void World::addRandomWallAnywhere()
{
	uniform_real_distribution<double> dist01(0,1);
	
	double x1 = sizeX_ * dist01(gen_);
	double y1 = sizeY_ * dist01(gen_);
	double x2 = sizeX_ * dist01(gen_);
	double y2 = sizeY_ * dist01(gen_);
	
	Wall wall(x1,y1,x2,y2);
	walls_.push_back(wall);
}


void World::addRandomWallOnSquareGrid()
{
	// subdivide the box in a Nx by Ny grid of smaller boxes
	// choose Nx,Ny such as the small boxes have a side length of about 5
	// we then place a wall randomly on the edges of this grid
	
	int Nx = int (sizeX_/5.0);
	int Ny = int (sizeY_/5.0);
	
	uniform_real_distribution<double> dist01(0,1);
	uniform_int_distribution<int> distX(0,Nx);
	uniform_int_distribution<int> distY(0,Ny);
	
	double dx = sizeX_*1.0/Nx;
	double dy = sizeY_*1.0/Ny;
	
	double x1 = dx * distX(gen_);
	double y1 = dy * distY(gen_);
	double x2 = x1;
	double y2 = y1;
	
	double r = dist01(gen_);
	
	if (r<0.5)
	{
		while (x2==x1) x2 = dx * distX(gen_);
	}
	else
	{
		while (y2==y1) y2 = dy * distY(gen_);
	}
	
	Wall wall(x1,y1,x2,y2);
	walls_.push_back(wall);
}



void World::placeBoids(vector<Boid*> boids)
{
	double vInit = 1e-5;
	uniform_real_distribution<double> dist01(0,1);
	
	for (int i=0; i<boids.size(); i++) boids_.push_back(boids[i]);
	
	for (int i=0; i<boids_.size(); i++)
	{
		boids_[i]->x = sizeX_*dist01(gen_);
		boids_[i]->y = sizeY_*dist01(gen_);
		boids_[i]->vx = vInit*(-0.5+dist01(gen_)); // to avoid problems when superposed
		boids_[i]->vy = vInit*(-0.5+dist01(gen_)); // and to initiate randommly-orientated movement
		
		boids_[i]->updateNeighbours(getBoids(), walls_);
	}
}



///////////////////////////// Time Integration /////////////////////////////


void World::advanceTime(double T, double dt)
{
	for (double t=0; t<T; t+=dt)
	{
		// Compute forces
		
		#pragma omp parallel for
		for (int i=0; i<boids_.size(); i++)
		{
			boids_[i]->resetForce();
			boids_[i]->computePhysicalForces(getBoids(), walls_);
			boids_[i]->computeBehaviouralForces(getBoids(), walls_);
		}
		
		// Integrate
		
		vector<Boid> boidsOld = getBoids();
		for (int i=0; i<boids_.size(); i++) boids_[i]->step(dt);
		collideWalls(boidsOld);
		
		// Update neighbour list (only once in a while)
		
		//if ((int (t/dt))%4==0)
		{
			#pragma omp parallel for
			for (int i=0; i<boids_.size(); i++)
			{
				boids_[i]->updateNeighbours(getBoids(), walls_);
			}
		}
	}
}


void World::collideWalls(const vector<Boid> &boidsOld)
{
	#pragma omp parallel for
	for (int i=0; i<boids_.size(); i++)
	{
		for (int j=0; j<walls_.size(); j++)
		{
			double x1 = walls_[j].x1;
			double y1 = walls_[j].y1;
			double x2 = walls_[j].x2;
			double y2 = walls_[j].y2;
			double x3 = boidsOld[i].x;
			double y3 = boidsOld[i].y;
			double x4 = boids_[i]->x;
			double y4 = boids_[i]->y;
			
			// Use Ray-Wall intersection code to detect collision
			
			double angle34 = angle(x3,y3,x4,y4);
			Ray ray(x3,y3,angle34);
			
			bool exists;
			double xInt, yInt;
			intersection(ray,walls_[j],xInt,yInt,exists);
			
			double d34 = distance(x3,y3,x4,y4);
			double d3Int = distance(x3,y3,xInt,yInt);
			bool isCollision = exists && d3Int<d34;
			
			if (isCollision)
			{
				// mirror velocities
				
				double vx = (boidsOld[i].vx+boids_[i]->vx)/2;
				double vy = (boidsOld[i].vy+boids_[i]->vy)/2;
				double V = sqrt(vx*vx+vy*vy);
				double angleV = angle(0,0,vx,vy);
				double angle12 = angle(x1,y1,x2,y2);
				
				double angleVMirror = 2*angle12-angleV;
				boids_[i]->vx = V*cos(angleVMirror*PI/180);
				boids_[i]->vy = V*sin(angleVMirror*PI/180);
				
				// mirror positions
				
				double angle34Mirror = 2*angle12-angle34;
				double dInt4 = d34-d3Int;
				boids_[i]->x = xInt + dInt4*cos(angle34Mirror*PI/180);
				boids_[i]->y = yInt + dInt4*sin(angle34Mirror*PI/180);
			}
		}
	}
}



