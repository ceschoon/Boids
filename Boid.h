////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                 Boid classes and related methods                       //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


#ifndef BOID_H
#define BOID_H

#include <vector>

using namespace std;


class Wall;
class Ray;


class Boid
{
	public: 
		Boid(double x, double y, double orientation);
		
		// state variables
		
		double x,y;
		double vx,vy;
		double fx,fy;            // mass=1 (force=acceleration)
		double orientation;      // fixed in the direction of speed
		vector<int> neighbours;
		
		// behaviour: affinities to various effects
		
		double b;   // cohesion force coefficient:    F_cohesion = -b*r             (behavioural, parallel)
		double s;   // alignment coefficient:        F_alignment = s*angleDiff      (behavioural, perpendicular)
		double w;   // wall avoidance coefficient:        F_wall = w*angleDiff^2    (behavioural, perpendicular)
		
		// physical properties
		
		double a;   // seperation force coefficent: F_seperation = a/r^2            (physical, towards any neighbour boid)
		double c;   // drag coefficient:                  F_drag = -1/2*c*v^2       (physical, parallel)
		
		double f;   // constant shift for parallel force
		double f1;  // amplitude for parallel behavioural forces
		double f2;  // amplitude for perpendicular behavioural forces
		
		double viewRange;
		double obstacleRange; // TODO: change the wall avoiding stuff
		double viewAngle;
		
		// Force-related Methods
		
		void resetForce();
		void computeBehaviouralForces(vector<Boid> &boids, vector<Wall> &walls);
		void computePhysicalForces(vector<Boid> &boids, vector<Wall> &walls);
		
		void computeCohesionForce(vector<Boid> &boids, double &fx_, double &fy_);
		void computeAlignmentForce(vector<Boid> &boids, double &fx_, double &fy_);
		void computeWallAvoidingForce(vector<Boid> &boids, vector<Wall> &walls, 
		                              double &fx_, double &fy_);
		
		void computeDragForce(vector<Boid> &boids, double &fx_, double &fy_);
		void computeSeparationForce(vector<Boid> &boids, double &fx_, double &fy_);
};


#endif //BOID_H
