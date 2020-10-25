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
#include "Physics.h"
#include "Boid.h"
#include "Math.h"
#include "Rendering.h"

using namespace std;


///////////////////////////////// World ////////////////////////////////////

// TODO: meant to be inherited (child class with custom wall/boid placement)

World::World(double sizeX, double sizeY)
: sizeX_(sizeX), sizeY_(sizeY)
{
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


//////////////////////// World rendering (call) ////////////////////////////

void World::render(sf::RenderWindow &window)
{
	double scaleX = window.getSize().x/sizeX_;
	double scaleY = window.getSize().y/sizeY_;
	
	// TODO: call render functions from "Rendering.h"
	
	renderWalls(window, walls_, scaleX, scaleY);
}

////////////////////////// Wall - Ray mechanics ////////////////////////////


void intersection(Ray ray, Wall wall, double &xInt, double &yInt, bool &exists)
{
	exists = true;
	
	double x1 = wall.x1;
	double y1 = wall.y1;
	double x2 = wall.x2;
	double y2 = wall.y2;
	double x3 = ray.originX;
	double y3 = ray.originY;
	double x4 = x3 + cos(ray.angle*PI/180); // whatever on the ray line
	double y4 = y3 + sin(ray.angle*PI/180); // whatever on the ray line
	
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
		if (cos(ray.angle*PI/180)>0 && xInt<x3) exists = false;
		// if ray goes to negative x, intersection must be < x3
		if (cos(ray.angle*PI/180)<0 && xInt>x3) exists = false;
		// if ray goes to positive y, intersection must be > y3
		if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
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
		if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
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
			if (cos(ray.angle*PI/180)>0 && xInt<x3) exists = false;
			// if ray goes to negative x, intersection must be < x3
			if (cos(ray.angle*PI/180)<0 && xInt>x3) exists = false;
			// if ray goes to positive y, intersection must be > y3
			if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
			// if ray goes to negative y, intersection must be < y3
			if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
		}
	}
}


////////////////////////////// Initialisation //////////////////////////////


void World::addRandomWall(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                          default_random_engine &gen)
{
	uniform_real_distribution<double> dist01(0,1);
	
	double x1 = boxSizeX * dist01(gen);
	double y1 = boxSizeY * dist01(gen);
	double x2 = boxSizeX * dist01(gen);
	double y2 = boxSizeY * dist01(gen);
	
	Wall wall(x1,y1,x2,y2);
	walls.push_back(wall);
}

void World::addRandomWallOnSquareGrid(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                                      default_random_engine &gen)
{
	// subdivide the box in a Nx by Ny grid of smaller boxes
	// choose Nx,Ny such as the small boxes have a side length of about 5
	// we then place a wall randomly on the edges of this grid
	
	int Nx = int (boxSizeX/5.0);
	int Ny = int (boxSizeY/5.0);
	
	uniform_real_distribution<double> dist01(0,1);
	uniform_int_distribution<int> distX(0,Nx);
	uniform_int_distribution<int> distY(0,Ny);
	
	double dx = boxSizeX*1.0/Nx;
	double dy = boxSizeY*1.0/Ny;
	
	double x1 = dx * distX(gen);
	double y1 = dy * distY(gen);
	double x2 = x1;
	double y2 = y1;
	
	double r = dist01(gen);
	
	if (r<0.5)
	{
		while (x2==x1) x2 = dx * distX(gen);
	}
	else
	{
		while (y2==y1) y2 = dy * distY(gen);
	}
	
	Wall wall(x1,y1,x2,y2);
	walls.push_back(wall);
}



void World::placeBoids(vector<Boid> &boids, double boxSizeX, double boxSizeY,
                       default_random_engine &gen)
{
	double vInit = 0.00001;
	uniform_real_distribution<double> dist01(0,1);
	
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x = boxSizeX*dist01(gen);
		boids[i].y = boxSizeY*dist01(gen);
		boids[i].vx = vInit*(-0.5+dist01(gen));	// to avoid problems when superposed
		boids[i].vy = vInit*(-0.5+dist01(gen)); // and to initiate randommly-orientated movement
	}
}



///////////////////////////// Time Integration /////////////////////////////


void World::advanceTime(vector<Boid> &boids, double dt)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x += boids[i].vx*dt + 0.5*boids[i].fx*dt*dt;
		boids[i].y += boids[i].vy*dt + 0.5*boids[i].fy*dt*dt;
		
		boids[i].vx += boids[i].fx*dt;
		boids[i].vy += boids[i].fy*dt;
	}
}


void World::collideWalls(vector<Boid> &boidsOld, vector<Boid> &boidsNew, vector<Wall> walls)
{
	for (int i=0; i<boidsNew.size(); i++)
	{		
		for (int j=0; j<walls.size(); j++)
		{
			double x1 = walls[j].x1;
			double y1 = walls[j].y1;
			double x2 = walls[j].x2;
			double y2 = walls[j].y2;
			double x3 = boidsOld[i].x;
			double y3 = boidsOld[i].y;
			double x4 = boidsNew[i].x;
			double y4 = boidsNew[i].y;
			
			// Use Ray-Wall intersection code to detect collision
			
			double angle34 = angle(x3,y3,x4,y4);
			Ray ray(x3,y3,angle34);
			
			bool exists;
			double xInt, yInt;
			intersection(ray,walls[j],xInt,yInt,exists);
			
			double d34 = distance(x3,y3,x4,y4);
			double d3Int = distance(x3,y3,xInt,yInt);
			bool isCollision = exists && d3Int<d34;
			
			if (isCollision)
			{
				// mirror velocities
				
				double vx = (boidsOld[i].vx+boidsNew[i].vx)/2;
				double vy = (boidsOld[i].vy+boidsNew[i].vy)/2;
				double V = sqrt(vx*vx+vy*vy);
				double angleV = angle(0,0,vx,vy);
				double angle12 = angle(x1,y1,x2,y2);
				
				double angleVMirror = 2*angle12-angleV;
				boidsNew[i].vx = V*cos(angleVMirror*PI/180);
				boidsNew[i].vy = V*sin(angleVMirror*PI/180);
				
				// mirror positions
				
				double angle34Mirror = 2*angle12-angle34;
				double dInt4 = d34-d3Int;
				boidsNew[i].x = xInt + dInt4*cos(angle34Mirror*PI/180);
				boidsNew[i].y = yInt + dInt4*sin(angle34Mirror*PI/180);
			}
		}
	}
}



