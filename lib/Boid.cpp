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
#include <chrono>
#include <omp.h>
#include <cmath>
#include "Boid.h"
#include "Physics.h"

using namespace std;
using namespace std::chrono;


// profiling (see Physics.h)
ProfilingData profData_;


Boid::Boid(double x_, double y_, double orientation_, double v_)
{
	x = x_;
	y = y_;
	vx = v_*cos(orientation_);
	vy = v_*sin(orientation_);
	fx = 0;
	fy = 0;
	neighbours = {};
	
	b = 0.1;             // default = 0.1
	s = 5.0/180;         // default = 5.0/180
	w = 25.0/180;        // default = 25.0/180
	
	a = 1;               // default = 1
	c = 1;               // default = 1
	
	f = 10;              // default = 10
	f1 = 10;             // default = 10
	f2 = 10;             // default = 10
	
	viewRange = 10;      // default = 10
	obstacleRange = 5;   // default = 5
	viewAngle = 120;     // default = 120
	
	doBoidRepulsion = true;
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
	if(doBoidRepulsion) computeSeparationForce(boids, fx_, fy_);
	
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
	computeWallAvoidingForce(boids, walls, fx_, fy_); // TODO: here is where most of the computation time for forces is spent
	
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
		double x3 = boids[neighbours[j]].getPosX();
		double y3 = boids[neighbours[j]].getPosY();
		
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



void Boid::computeWallAvoidingForce(const vector<Boid> &boids, 
                                    const vector<Wall> &walls,
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
		double x2 = boids[neighbours[j]].getPosX();
		double y2 = boids[neighbours[j]].getPosY();
		double r = distance(x,y,x2,y2);
		double angle12 = angle(x2,y2,x,y);
		
		// no test for r<range as it is done in neighbour list construction
		double F = a/(r*r);
		fx_ += F*cos(angle12*PI/180);
		fy_ += F*sin(angle12*PI/180);
	}
}






//////////////////////////// Time evolution ////////////////////////////////


void Boid::step(double dt)
{
	x += vx*dt + 0.5*fx*dt*dt;
	y += vy*dt + 0.5*fy*dt*dt;
	
	vx += fx*dt;
	vy += fy*dt;
}







/////////////////////// Neighbour tests and updates ////////////////////////


bool Boid::isNeighbour (int index) const
{
	bool isIt = false;
	
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


// TODO Loose a factor 2 here because of this function being a member of the
//      boid class. Otherwise we could simply call it on j>i and update the 
//      two boids at once.
void Boid::updateNeighbours(const vector<Boid> &boids, const vector<Wall> &walls)
{
	findWallsInView(walls);
	vector<int> neighboursNew = {};
	
	for (int j=0; j<boids.size(); j++)
	{
		// check that the new boid is not the current one
		if (boids[j].getPosX()==x && boids[j].getPosY()==y) continue;
		
		double xj = boids[j].getPosX();
		double yj = boids[j].getPosY();
		
		// distance condition
		double dij = distance(x,y,xj,yj);
		if (dij > viewRange) continue;
		
		// visibility condition (angle)
		double angleij = angle(x,y,xj,yj);
		if (abs(angleDifference(angleij,orientation()))>viewAngle) continue;
		
		//?TODO: Optimise visibility checks by precomputing visibility 
		//       between chunks on the map. Keep track of the chunk the 
		//       boids are in. This chunk method can also be used to 
		//       optimise the distance and angle checks.
		
		// visibility condition (no wall)
		bool viewObstructed = false;
		Ray ray(x,y,angleij);
		
		for (int k=0; k<wallsInView.size(); k++)
		{
			bool exists;
			double xInt,yInt;
			intersection(ray,wallsInView[k],xInt,yInt,exists);
			
			if ( exists && distance(x,y,xInt,yInt) < distance(x,y,xj,yj) )
			{
				viewObstructed = true;
				break;
			}
		}
		
		// count as neightbour if all conditions are satisfied
		if (!viewObstructed) neighboursNew.push_back(j);
	}
	
	neighbours = neighboursNew;
}


// 1. Check if wall crosses one of the view rays at the largest angle.
//    If no, continue. If yes, record this wall.
// 2. Check if point on wall line that is the closest to the boid is in range
//    If no, continue. If yes, 
//    2.2. Check if this wall extremity is within view angle
//         If no, continue. If yes, record this wall.

void Boid::findWallsInView(const vector<Wall> &walls)
{
	vector<Wall> wallsInViewNew = {};
	
	for (int i=0; i<walls.size(); i++)
	{
		// Do not count border walls
		if (walls[i].isABorder) continue;
		
		// Check if wall crosses one of the view rays at the largest angle
		
		bool exists; double xInt,yInt;
		
		Ray ray1(x,y,orientation()-viewAngle);
		intersection(ray1,walls[i],xInt,yInt,exists);
		if (exists) wallsInViewNew.push_back(walls[i]);
		
		Ray ray2(x,y,orientation()+viewAngle);
		intersection(ray2,walls[i],xInt,yInt,exists);
		if (exists) wallsInViewNew.push_back(walls[i]);
		
		// Find point on wall line that is the closest to the boid
		// Check if it is in range (distance and angle)
		
		double xw, yw;
		walls[i].findClosestPoint(x,y,xw,yw);
		
		double dw = distance(x,y,xw,yw);
		if (dw < viewRange)
		{
			double anglew = angle(x,y,xw,yw);
			if (abs(angleDifference(anglew,orientation()))<viewAngle)
				wallsInViewNew.push_back(walls[i]);
		}
	}
	
	wallsInView = wallsInViewNew;
}



