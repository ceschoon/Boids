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

using namespace std;

uniform_real_distribution<double> dist01(0,1);



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


void addRandomWall(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                   default_random_engine &gen)
{
	double x1 = boxSizeX * dist01(gen);
	double y1 = boxSizeY * dist01(gen);
	double x2 = boxSizeX * dist01(gen);
	double y2 = boxSizeY * dist01(gen);
	
	Wall wall(x1,y1,x2,y2);
	walls.push_back(wall);
}

void addRandomWallOnSquareGrid(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                               default_random_engine &gen)
{
	// subdivide the box in a Nx by Ny grid of smaller boxes
	// choose Nx,Ny such as the small boxes have a side length of about 5
	// we then place a wall randomly on the edges of this grid
	
	int Nx = int (boxSizeX/5.0);
	int Ny = int (boxSizeY/5.0);
	
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



void placeBoids(vector<Boid> &boids, double boxSizeX, double boxSizeY,
                default_random_engine &gen)
{
	double vInit = 0.00001;
	
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x = boxSizeX*dist01(gen);
		boids[i].y = boxSizeY*dist01(gen);
		boids[i].orientation = 360*dist01(gen);
		boids[i].vx = vInit*(-0.5+dist01(gen));	// to avoid problems when superposed
		boids[i].vy = vInit*(-0.5+dist01(gen)); // and to initiate randommly-orientated movement
	}
}



///////////////////////////// Time Integration /////////////////////////////


void advanceTime(vector<Boid> &boids, double dt)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x += boids[i].vx*dt + 0.5*boids[i].fx*dt*dt;
		boids[i].y += boids[i].vy*dt + 0.5*boids[i].fy*dt*dt;
		
		boids[i].vx += boids[i].fx*dt;
		boids[i].vy += boids[i].fy*dt;
	}
}

void orientationWithSpeed(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		double orientation1 = boids[i].orientation;
		double angleSpeed = angle(0,0,boids[i].vx,boids[i].vy);
		boids[i].orientation = angleSpeed;
	}
}

void collideWalls(vector<Boid> &boidsOld, vector<Boid> &boidsNew, vector<Wall> walls)
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




//////////////////////////////// Forces ////////////////////////////////////


void resetForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].fx = 0;
		boids[i].fy = 0;
	}
}

void drivingForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].fx += boids[i].f*cos(boids[i].orientation*PI/180);
		boids[i].fy += boids[i].f*sin(boids[i].orientation*PI/180);
	}
}

void drag(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		double speed2 = boids[i].vx*boids[i].vx+boids[i].vy*boids[i].vy;
		double angleSpeed = angle(0,0,boids[i].vx,boids[i].vy);
		
		boids[i].fx -= 0.5*boids[i].c*speed2*cos(angleSpeed*PI/180);
		boids[i].fy -= 0.5*boids[i].c*speed2*sin(angleSpeed*PI/180);
	}
}

void separation(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		
		double x1 = boids[i].x;
		double y1 = boids[i].y;
			
		for (int j=0; j<neighbours.size(); j++)
		{
			double x2 = boids[neighbours[j]].x;
			double y2 = boids[neighbours[j]].y;
			double r = distance(x1,y1,x2,y2);
			double angle12 = angle(x2,y2,x1,y1);
			
			// no test for r<range as it is done in the neighbour list
			double F = boids[i].a/(r*r);
			boids[i].fx += F*cos(angle12*PI/180);
			boids[i].fy += F*sin(angle12*PI/180);	
		}
	}
}

void cohesion(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		if (neighbours.size()==0) continue;
		
		double x1 = boids[i].x;
		double y1 = boids[i].y;
		
		// gravity center of neighbour boids
		double x2 = 0;
		double y2 = 0;
		
		for (int j=0; j<neighbours.size(); j++)
		{
			double x3 = boids[neighbours[j]].x;
			double y3 = boids[neighbours[j]].y;
			
			x2 += x3/neighbours.size();
			y2 += y3/neighbours.size();
		}
		
		double r = distance(x1,y1,x2,y2);
		double angle12 = angle(x2,y2,x1,y1);
		
		// no test for r<range as it is done in the neighbour list
		double F = -boids[i].b*r;
		
		boids[i].fx += F*cos(angle12*PI/180);
		boids[i].fy += F*sin(angle12*PI/180);
	}
}

void alignment(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		if (neighbours.size()==0) continue;
		
		int N = neighbours.size();
		double orientation1 = boids[i].orientation;
		double avgOrientation = 0;
		
		for (int j=0; j<neighbours.size(); j++)
		{
			avgOrientation += angle360(boids[neighbours[j]].orientation)/N;
		}
		
		double angleDiff = angleDifference(avgOrientation,orientation1);
		double F = boids[i].s*angleDiff;
		
		boids[i].fx += F*cos(orientation1*PI/180+PI/2);
		boids[i].fy += F*sin(orientation1*PI/180+PI/2);
	}
}

void avoidWalls(vector<Boid> &boids, vector<Wall> walls)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		double orientation = boids[i].orientation;
			
		bool foundFreeRay = false;
		bool outOfRays = false;
		Ray ray(boids[i].x, boids[i].y, boids[i].orientation);
		
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
				bool obstructed = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				
				// check if neighbour rays also intersect the wall
				double delta = 15; //default 15
				Ray rayLeft(ray.originX, ray.originY, ray.angle-delta);
				intersection(rayLeft, walls[j], xInt, yInt, exists);
				bool obstructedLeft = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				Ray rayRight(ray.originX, ray.originY, ray.angle+delta);
				intersection(rayRight, walls[j], xInt, yInt, exists);
				bool obstructedRight = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				
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
				
				// alternate looking left and right of the current orientation
				// TODO: randomise instead?
				if (angleDifference(ray.angle,orientation) >= 0) 
					ray.angle = orientation - (ray.angle-orientation) - angleStep;
				else if (angleDifference(ray.angle,orientation) < 0)  
					ray.angle = orientation - (ray.angle-orientation) + angleStep;
			}
			
			if (abs(angleDifference(ray.angle,orientation)) > boids[i].viewAngle)
				outOfRays = true;
		}
		
		if (foundFreeRay == true) 
		{
			double angleDiff = angleDifference(ray.angle,orientation);
			double F = boids[i].w*angleDiff;
			
			boids[i].fx += F*cos(orientation*PI/180+PI/2);
			boids[i].fy += F*sin(orientation*PI/180+PI/2);
		}
		else // the boid feels trapped and turns on himself
		{
			double angleDiff = 180;
			double F = boids[i].w*angleDiff;
			
			boids[i].fx += F*cos(orientation*PI/180+PI/2);
			boids[i].fy += F*sin(orientation*PI/180+PI/2);
			
			//boids[i].orientation += 180;
		}
	}
}

void mouseWorshipping(vector<Boid> &boids, vector<Wall> walls, double mouseX, double mouseY)
{
	for (int i=0; i<boids.size(); i++)
	{			
		// distance condition
		double d = distance(boids[i].x,boids[i].y,mouseX,mouseY);
		
		// visibility condition (angle)
		double angleMouse = angle(boids[i].x,boids[i].y,mouseX,mouseY);
		
		// visibility condition (no wall)
		bool viewObstructed = false;
		Ray ray(boids[i].x,boids[i].y,angleMouse);
		
		for (int k=0; k<walls.size(); k++)
		{
			bool exists;
			double xInt,yInt;
			intersection(ray,walls[k],xInt,yInt,exists);
			if (exists && distance(boids[i].x,boids[i].y,xInt,yInt)
				<distance(boids[i].x,boids[i].y,mouseX,mouseY)) 
			{
				viewObstructed = true;
				break;
			}
		}
		
		if (d < boids[i].range && 
			abs(angleDifference(angleMouse,boids[i].orientation))<boids[i].viewAngle &&
			!viewObstructed)
		{
			boids[i].fx += boids[i].m*cos(angleMouse*PI/180);
			boids[i].fy += boids[i].m*sin(angleMouse*PI/180);
		}
	}
}

void capForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		double F = sqrt(boids[i].fx*boids[i].fx+boids[i].fy*boids[i].fy);
		double angleF = angle(0,0,boids[i].fx,boids[i].fy);
		
		double Fcap = 100; // default 100
		if (F>Fcap) 
		{
			boids[i].fx = Fcap*cos(angleF*PI/180);
			boids[i].fy = Fcap*sin(angleF*PI/180);
		}
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
				
				if (d < boids[i].range && 
					abs(angleDifference(angleij,boids[i].orientation))<boids[i].viewAngle &&
					!viewObstructed)
				{
					neighbours.push_back(j);
				}
			}
		}
		
		boids[i].neighbours = neighbours;
	}
}