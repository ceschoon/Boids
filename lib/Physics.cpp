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
#include <iomanip>
#include <fstream>
#include <omp.h>
#include <chrono>
#include "Physics.h"
#include "Boid.h"
#include "Math.h"
#include "Rendering.h"
#include "hard_spheres.h"

using namespace std;
using namespace std::chrono;



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
	
	Wall wallBorder0(0,0,0,sizeY_,true);
	Wall wallBorder1(0,sizeY_,sizeX_,sizeY_,true);
	Wall wallBorder2(sizeX_,sizeY_,sizeX_,0,true);
	Wall wallBorder3(sizeX_,0,0,0,true);
	
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


/////////////////////////////// Profiling //////////////////////////////////

void World::printProfilingData(int argc, char **argv)
{
	ofstream file("profiling.out");
	file << fixed << setprecision(3);
	
	file << "================= Profiling Data ================" << endl;
	file << "From invoking:" << endl;
	for (int i=0; i<argc; i++) file << argv[i] << " "; file << endl;
	file << "=================================================" << endl;
	file << endl;
	file << "calls for render              " << setw(8) << profData_.calls_render << endl;
	file << "calls for step                " << setw(8) << profData_.calls_step << endl;
	file << "calls for forces (all)        " << setw(8) << profData_.calls_allforces << endl;
	file << "calls for updateNeighbours    " << setw(8) << profData_.calls_updateNeighbours << endl;
	file << "calls for collideWalls        " << setw(8) << profData_.calls_collideWalls << endl;
	file << endl;
	file << "avg time for render              " << setw(12) << profData_.avg_time_render << " microseconds" << endl;
	file << "avg time for step                " << setw(12) << profData_.avg_time_step << " microseconds" << endl;
	file << "avg time for forces (all)        " << setw(12) << profData_.avg_time_allforces << " microseconds" << endl;
	file << "avg time for updateNeighbours    " << setw(12) << profData_.avg_time_updateNeighbours << " microseconds" << endl;
	file << "avg time for collideWalls        " << setw(12) << profData_.avg_time_collideWalls << " microseconds" << endl;
	file << endl;
	file << "tot time for render              " << setw(12) << 1e-6*profData_.avg_time_render*profData_.calls_render << " seconds" << endl;
	file << "tot time for step                " << setw(12) << 1e-6*profData_.avg_time_step*profData_.calls_step << " seconds" << endl;
	file << "tot time for forces (all)        " << setw(12) << 1e-6*profData_.avg_time_allforces*profData_.calls_allforces << " seconds" << endl;
	file << "tot time for updateNeighbours    " << setw(12) << 1e-6*profData_.avg_time_updateNeighbours*profData_.calls_updateNeighbours << " seconds" << endl;
	file << "tot time for collideWalls        " << setw(12) << 1e-6*profData_.avg_time_collideWalls*profData_.calls_collideWalls << " seconds" << endl;
	file << endl;
}


//////////////////////// World rendering (call) ////////////////////////////

void World::render(sf::RenderWindow &window)
{
	steady_clock::time_point start = steady_clock::now();
	
	double scaleX = window.getSize().x/sizeX_;
	double scaleY = window.getSize().y/sizeY_;
	
	renderWalls(window, walls_, scaleX, scaleY);
	renderBoidsAsPoints(window, boids_, scaleX, scaleY, -1);
	renderBoidsAsTriangles(window, boids_, scaleX, scaleY);
	
	steady_clock::time_point stop = steady_clock::now();
	int time_us = duration_cast<microseconds>(stop-start).count();
	
	profData_.calls_render ++; int n=profData_.calls_render;
	profData_.avg_time_render += (time_us-profData_.avg_time_render)/n;
}


void World::renderDebug(sf::RenderWindow &window, int i, bool doForces)
{
	steady_clock::time_point start = steady_clock::now();
	
	double scaleX = window.getSize().x/sizeX_;
	double scaleY = window.getSize().y/sizeY_;
	
	renderWalls(window, walls_, scaleX, scaleY);
	renderWallsInView(window, boids_, walls_, scaleX, scaleY, i);
	renderBoidsHighlight(window, boids_, scaleX, scaleY, i);
	if (doForces) renderForces(window, boids_, scaleX, scaleY);
	
	steady_clock::time_point stop = steady_clock::now();
	int time_us = duration_cast<microseconds>(stop-start).count();
	
	profData_.calls_render ++; int n=profData_.calls_render;
	profData_.avg_time_render += (time_us-profData_.avg_time_render)/n;
}


////////////////////////// Wall - Ray mechanics ////////////////////////////


void intersection(const Ray &ray, const Wall &wall, double &xInt, double &yInt, bool &exists)
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

/*
// Piece of code to artificially double the time this function takes
void intersection(const Ray &ray, const Wall &wall, double &xInt, double &yInt, bool &exists)
{
	intersection_(ray, wall, xInt, yInt, exists);
	intersection_(ray, wall, xInt, yInt, exists);
}
*/


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
	cout << "Placing boids in the World" << endl;
	
	double vInit = 1e-5;
	uniform_real_distribution<double> dist01(0,1);
	
	for (int i=0; i<boids.size(); i++) boids_.push_back(boids[i]);
	
	bool valid_config = false;
	int max_attempts_config = 1000;
	int num_attempts_config = 0;
	
	while (!valid_config)
	{
		valid_config = true;
		for (int i=0; i<boids_.size(); i++)
		{
			bool overlap = true;
			double x,y;
			
			int max_attempts_new_sphere = 1000;
			int num_attempts_new_sphere = 0;
			
			while (overlap)
			{
				overlap = false;
				x = boids[i]->getRadius() + dist01(gen_)*(sizeX_-2*boids[i]->getRadius());
				y = boids[i]->getRadius() + dist01(gen_)*(sizeY_-2*boids[i]->getRadius());
				
				for (int j=0; j<i; j++)
				{
					double d2ij = (x-boids_[j]->x)*(x-boids_[j]->x)
						        + (y-boids_[j]->y)*(y-boids_[j]->y);
					
					double dmin = boids[i]->getRadius() + boids[j]->getRadius();
					
					if (d2ij < dmin*dmin) overlap = true;
				}
				
				num_attempts_new_sphere ++;
				if (num_attempts_new_sphere>=max_attempts_new_sphere) 
				{
					valid_config = false;
					break;
				}
			}
			
			if (!valid_config) break;
			
			boids_[i]->x = x;
			boids_[i]->y = y;
			boids_[i]->vx = vInit*(-0.5+dist01(gen_));
			boids_[i]->vy = vInit*(-0.5+dist01(gen_));
			
			boids_[i]->updateNeighbours(getBoids(), walls_, i);
		}
		
		num_attempts_config ++;
		if (num_attempts_config>=max_attempts_config) 
		{
			throw runtime_error("Exceeded max allowed attempts while placing boids in the world");
		}
	}
	
	cout << "  done in " << num_attempts_config << " attempts" << endl;
}



///////////////////////////// Time Integration /////////////////////////////


void World::advanceTime(double T, double dt_max)
{
	double t  = 0;
	double dt = dt_max;
	
	while (t<T)
	{
		// Do not exceed max time
		// Update time with dt_max so that the loop can end
		
		t += dt_max;
		if (t+dt>T) dt = T-t;
		
		// Compute forces
		
		steady_clock::time_point start  = steady_clock::now();
		steady_clock::time_point start2 = steady_clock::now();
		
		vector<Boid> boidsCopy = getBoids();
		
		#pragma omp parallel for
		for (int i=0; i<boids_.size(); i++)
		{
			boids_[i]->resetForce();
			boids_[i]->computePhysicalForces(boidsCopy, walls_);
			boids_[i]->computeBehaviouralForces(boidsCopy, walls_);
		}
		
		{
			// profiling: timing forces
			steady_clock::time_point stop2 = steady_clock::now();
			int time_us = duration_cast<microseconds>(stop2-start2).count();
			
			profData_.calls_allforces +=boids_.size(); int n=profData_.calls_allforces/boids_.size();
			profData_.avg_time_allforces += (time_us/boids_.size()-profData_.avg_time_allforces)/n;
		}
		
		// Integrate positions based on Hard Spheres dynamics
		// This is consistent with Euler step as the change is linear in dt
		
		double L[3] = {sizeX_, sizeY_, 10.0};
		vector<double> hs_r, hs_m, hs_x, hs_y, hs_z, hs_vx, hs_vy, hs_vz;
		
		for (int i=0; i<boids_.size(); i++)
		{
			hs_r.push_back( boids_[i]->getRadius() );
			hs_m.push_back( boids_[i]->getMass() );
			
			hs_x.push_back( boids_[i]->x );
			hs_y.push_back( boids_[i]->y );
			hs_z.push_back( 5.0 );
			
			hs_vx.push_back( boids_[i]->vx );
			hs_vy.push_back( boids_[i]->vy );
			hs_vz.push_back( 0.0 );
		}
		
		MyHardSpheres hs_integrator(boids_.size(), L, hs_r, hs_m, hs_x, hs_y, hs_z, hs_vx, hs_vy, hs_vz);
		hs_integrator.advance_time(dt);
		
		//for (int i=0; i<boids_.size(); i++) boids_[i]->step(dt);
		//collideWalls(boidsCopy); // here the copy is the old state
		
		for (int i=0; i<boids_.size(); i++)
		{
			boids_[i]->x = hs_integrator.get_x(i);
			boids_[i]->y = hs_integrator.get_y(i);
			
			boids_[i]->vx = hs_integrator.get_vx(i);
			boids_[i]->vy = hs_integrator.get_vy(i);
		}
		
		// Integrate velocities with computed forces
		
		for (int i=0; i<boids_.size(); i++) boids_[i]->step_v(dt);
		
		
		// Update neighbour list
		
		start2 = steady_clock::now();
		
		boidsCopy = getBoids();
		
		#pragma omp parallel for
		for (int i=0; i<boids_.size(); i++)
		{
			boids_[i]->findWallsInView(walls_);
			boids_[i]->updateNeighbours(boidsCopy, walls_, i);
		}
		
		{
			// profiling: timing neighbour updates
			steady_clock::time_point stop2 = steady_clock::now();
			int time_us = duration_cast<microseconds>(stop2-start2).count();
			
			profData_.calls_updateNeighbours += boids_.size(); int n=profData_.calls_updateNeighbours/boids_.size();
			profData_.avg_time_updateNeighbours += (time_us/boids_.size()-profData_.avg_time_updateNeighbours)/n;
		}
		
		// profiling: timing step
		steady_clock::time_point stop = steady_clock::now();
		int time_us = duration_cast<microseconds>(stop-start).count();
		
		profData_.calls_step ++; int n=profData_.calls_step;
		profData_.avg_time_step += (time_us-profData_.avg_time_step)/n;
	}
}


void World::collideWalls(const vector<Boid> &boidsOld)
{
	steady_clock::time_point start = steady_clock::now();
	
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
	
	// profiling: timing
	steady_clock::time_point stop = steady_clock::now();
	int time_us = duration_cast<microseconds>(stop-start).count();
	
	profData_.calls_collideWalls ++; int n=profData_.calls_collideWalls;
	profData_.avg_time_collideWalls += (time_us-profData_.avg_time_collideWalls)/n;
}



//////////////////////////////// Wall methods //////////////////////////////


void Wall::findClosestPoint(double x, double y, double &xw, double &yw) const
{
	double a[2] = {x2-x1,y2-y1};
	double b[2] = {x -x1,y -y1};
	
	double dotab = a[0]*b[0]+a[1]*b[1];
	double dotaa = a[0]*a[0]+a[1]*a[1];
	double dotbb = b[0]*b[0]+b[1]*b[1];
	
	// projection not on the wall and behind (x1,y1)
	if (dotab<0) 
	{
		xw = x1;
		yw = y1;
	}
	
	// projection not on the wall and behind (x2,y2)
	else if (dotab>dotaa) 
	{
		xw = x2;
		yw = y2;
	}
	
	// projection on the wall
	else
	{
		double lambda = dotab/dotaa;
		xw = x1 + a[0] * lambda;
		yw = y1 + a[1] * lambda;
	}
}








