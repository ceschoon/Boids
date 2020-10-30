////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Source file for the implementation of                    //
//                 Boid classes and related methods                       //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



#include <iostream>
#include <omp.h>
#include <cmath>
#include "Boid.h"
#include "Physics.h"

using namespace std;



Boid::Boid(double x_, double y_, double orientation_, double v_)
{
	x = x_;
	y = y_;
	vx = v_*cos(orientation_);
	vy = v_*sin(orientation_);
	fx = 0;
	fy = 0;
	neighbours = {};
	
	b = 1;               // default = 1
	s = 1.0/180*10 ;     // default = 1.0/180*10
	w = 1.0/180*100;     // default = 1.0/180*100
	
	a = 10;              // default = 10
	c = 1;               // default = 1
	
	f = 10;              // default = 10
	f1 = 5;              // default = 5
	f2 = 50;             // default = 50
	
	viewRange = 10;      // default = 10
	obstacleRange = 5;   // default = 5
	viewAngle = 120;     // default = 120
}



void Boid::resetForce()
{
	fx = 0;
	fy = 0;
}



void Boid::computePhysicalForces(const vector<Boid> &boids, const vector<Wall> &walls)
{
	// compute and sum all physical forces
	
	double fx_ = 0; double fy_ = 0;
	
	computeDragForce(boids, fx_, fy_);
	computeSeparationForce(boids, fx_, fy_);
	
	// update the Boid forces
	
	fx += fx_;
	fy += fy_;
}



void Boid::computeBehaviouralForces(const vector<Boid> &boids, const vector<Wall> &walls)
{
	// compute and sum all behavioural forces
	
	double fx_ = 0; double fy_ = 0;
	
	computeCohesionForce(boids, fx_, fy_);
	computeAlignmentForce(boids, fx_, fy_);
	computeWallAvoidingForce(boids, walls, fx_, fy_);
	
	// decompose in the frame of the Boid
	
	double f1_ =  fx_*cos(orientation()/180*PI) + fy_*sin(orientation()/180*PI);
	double f2_ = -fx_*sin(orientation()/180*PI) + fy_*cos(orientation()/180*PI);
	
	// renormalise each component
	
	f1_ = f + f1*sigmoid(f1_);   // parallel component
	f2_ = f2*sigmoid(f2_);       // perpendicular component
	
	// update the Boid forces in the fixed xy-frame
	
	fx += f1_*cos(orientation()/180*PI) - f2_*sin(orientation()/180*PI);
	fy += f1_*sin(orientation()/180*PI) + f2_*cos(orientation()/180*PI);
}



void Boid::computeCohesionForce(const vector<Boid> &boids, double &fx_, double &fy_)
{
	int N = neighbours.size();
	if (N==0) return;
	
	// gravity center of neighbour boids
	double x2 = 0;
	double y2 = 0;
	
	for (int j=0; j<neighbours.size(); j++)
	{
		double x3 = boids[neighbours[j]].x;
		double y3 = boids[neighbours[j]].y;
		
		x2 += x3/N;
		y2 += y3/N;
	}
	
	double r = distance(x,y,x2,y2);
	double angle12 = angle(x2,y2,x,y);
	
	// no test for r<range as it is done in neighbour list construction
	double F = -b*r;
	
	fx_ += F*cos(angle12*PI/180);
	fy_ += F*sin(angle12*PI/180);
}



void Boid::computeAlignmentForce(const vector<Boid> &boids, double &fx_, double &fy_)
{
	if (neighbours.size()==0) return;
	
	int N = neighbours.size();
	double avgOrientation = 0;
	
	for (int j=0; j<neighbours.size(); j++)
	{
		avgOrientation += angle360(boids[neighbours[j]].orientation())/N;
	}
	
	double angleDiff = angleDifference(avgOrientation,orientation());
	double F = s*angleDiff;
	
	fx_ += F*cos(orientation()*PI/180+PI/2);
	fy_ += F*sin(orientation()*PI/180+PI/2);
}



void Boid::computeWallAvoidingForce(const vector<Boid> &boids, const vector<Wall> &walls,
                                    double &fx_, double &fy_)
{
	bool foundFreeRay = false;
	bool outOfRays = false;
	Ray ray(x, y, orientation());
	
	while (foundFreeRay == false && outOfRays == false)
	{	 
		foundFreeRay = true;
		for (int j=0; j<walls.size(); j++)
		{
			// check if ray intersect the wall
			double xInt = 0;
			double yInt = 0;
			bool exists = false;
			intersection(ray, walls[j], xInt, yInt, exists);
			bool obstructed = exists && distance(ray.originX,ray.originY,xInt,yInt)<obstacleRange;
			
			// check if neighbour rays also intersect the wall
			double delta = 15; //default 15
			Ray rayLeft(ray.originX, ray.originY, ray.angle-delta);
			intersection(rayLeft, walls[j], xInt, yInt, exists);
			bool obstructedLeft = exists && distance(ray.originX,ray.originY,xInt,yInt)<obstacleRange;
			Ray rayRight(ray.originX, ray.originY, ray.angle+delta);
			intersection(rayRight, walls[j], xInt, yInt, exists);
			bool obstructedRight = exists && distance(ray.originX,ray.originY,xInt,yInt)<obstacleRange;
			
			if (obstructed || obstructedLeft || obstructedRight)
			{
				foundFreeRay = false;
				break;
			}	
		}
		
		if (foundFreeRay == false)
		{
			// search at a new angle
			double angleStep = 3; //default 3
			
			// alternate looking left and right of the current orientation()
			// TODO: randomise instead?
			if (angleDifference(ray.angle,orientation()) >= 0) 
				ray.angle = orientation() - (ray.angle-orientation()) - angleStep;
			else if (angleDifference(ray.angle,orientation()) < 0)  
				ray.angle = orientation() - (ray.angle-orientation()) + angleStep;
		}
		
		if (abs(angleDifference(ray.angle,orientation())) > viewAngle)
			outOfRays = true;
	}
	
	if (foundFreeRay == true) 
	{
		double angleDiff = angleDifference(ray.angle,orientation());
		double F = w*angleDiff;
		
		fx_ += F*cos(orientation()*PI/180+PI/2);
		fy_ += F*sin(orientation()*PI/180+PI/2);
	}
	else // the boid feels trapped and turns on himself
	{
		double angleDiff = 180;
		double F = w*angleDiff;
		
		fx_ += F*cos(orientation()*PI/180+PI/2);
		fy_ += F*sin(orientation()*PI/180+PI/2);
	}
}



void Boid::computeDragForce(const vector<Boid> &boids, double &fx_, double &fy_)
{
	double speed2 = vx*vx + vy*vy;
	
	fx_ -= 0.5*c*speed2*cos(orientation()*PI/180);
	fy_ -= 0.5*c*speed2*sin(orientation()*PI/180);
}



void Boid::computeSeparationForce(const vector<Boid> &boids, double &fx_, double &fy_)
{
	for (int j=0; j<neighbours.size(); j++)
	{
		double x2 = boids[neighbours[j]].x;
		double y2 = boids[neighbours[j]].y;
		double r = distance(x,y,x2,y2);
		double angle12 = angle(x2,y2,x,y);
		
		// no test for r<range as it is done in neighbour list construction
		double F = a/(r*r);
		fx_ += F*cos(angle12*PI/180);
		fy_ += F*sin(angle12*PI/180);
	}
}



/////////////////////// Neighbour tests and updates ////////////////////////



bool isNeighbour(int index, Boid boid)
{
	bool isIt = false;
	vector<int> neighbours = boid.neighbours;
	
	for (int i=0; i<neighbours.size(); i++)
	{
		if (index == neighbours[i]) 
		{
			isIt = true;
			break;
		}
	}
	
	return isIt;
}



void updateNeighbours(vector<Boid> &boids, vector<Wall> walls)
{
	#pragma omp parallel for
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = {};
		
		for (int j=0; j<boids.size(); j++)
		{
			if (i!=j)
			{
				// distance condition
				double d = distance(boids[i].x,boids[i].y,boids[j].x,boids[j].y);
				
				// visibility condition (angle)
				double angleij = angle(boids[i].x,boids[i].y,boids[j].x,boids[j].y);
				
				// visibility condition (no wall)
				bool viewObstructed = false;
				Ray ray(boids[i].x,boids[i].y,angleij);
				
				for (int k=0; k<walls.size(); k++)
				{
					bool exists;
					double xInt,yInt;
					intersection(ray,walls[k],xInt,yInt,exists);
					if (exists && distance(boids[i].x,boids[i].y,xInt,yInt)
						<distance(boids[i].x,boids[i].y,boids[j].x,boids[j].y)) 
					{
						viewObstructed = true;
						break;
					}
				}
				
				if (d < boids[i].viewRange && 
					abs(angleDifference(angleij,boids[i].orientation()))<boids[i].viewAngle &&
					!viewObstructed)
				{
					neighbours.push_back(j);
				}
			}
		}
		
		boids[i].neighbours = neighbours;
	}
}



