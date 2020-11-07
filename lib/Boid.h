////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                 Boid classes and related methods                       //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


// TODO: bad naming convention (member var shoud be underscored instead)
// TODO: ? move physical forces in physics class ?
// TODO: ? add boid-wall repulsive potential ?


#ifndef BOID_H
#define BOID_H

#include <vector>
#include "Math.h"

using namespace std;


class Wall;
class Ray;


class Boid
{
	public: 
		
		Boid(double x_, double y_, double orientation_, double v_);
		
		// Accessor-ish
		
		double getPosX() const {return x;}
		double getPosY() const {return y;}
		double orientation() const {return angle(0,0,vx,vy);}
		
		// Force-related Methods
		
		void resetForce();
		virtual void computeBehaviouralForces(const vector<Boid> &boids, const vector<Wall> &walls);
		void computePhysicalForces(const vector<Boid> &boids, const vector<Wall> &walls);
		
		virtual void computeCohesionForce(const vector<Boid> &boids, double &fx_, double &fy_);
		virtual void computeAlignmentForce(const vector<Boid> &boids, double &fx_, double &fy_);
		virtual void computeWallAvoidingForce(const vector<Boid> &boids, const vector<Wall> &walls, 
		                                      double &fx_, double &fy_);
		
		void computeDragForce(const vector<Boid> &boids, double &fx_, double &fy_);
		void computeSeparationForce(const vector<Boid> &boids, double &fx_, double &fy_);
		
		// Time evolution
		void step(double dt);
		
		// Neighbour detection
		
		bool isNeighbour(int index);
		void updateNeighbours(const vector<Boid> &boids, const vector<Wall> walls);
		
		// state variables
		// (difficult to encapsulate, cleaner as public)
		
		double x,y;
		double vx,vy;
		double fx,fy;            // mass=1 (force=acceleration)
		vector<int> neighbours;
		
	protected:
	
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
		double obstacleRange;
		double viewAngle;
};



#endif //BOID_H
