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



class Boid
{
	public: 
		Boid(double x, double y, double orientation);
		
		double x,y;
		double vx,vy;
		double fx,fy;	// mass=1 (force=acceleration)
		double orientation;
		
		double a;	// seperation force coefficent: F_seperation = a/r^2			(towards target)
		double b;	// cohesion force coefficient:    F_cohesion = -b*r				(towards from target)
		double s;	// alignment coefficient:		 F_alignment = s*angleDiff^2	(perpendicular to the movement)
		double f;	// driving force:                  F_driving = f				(along the movement)
		double c;	// drag coefficient:                  F_drag = -1/2*c*v^2		(along the movement)
		double m;	// mouse affinity					 F_mouse = m				(towards mouse)
		double w;	// wall avoidance coefficient:        F_wall = w*angleDiff^2	(perpendicular to the movement)
		
		double range;
		double obstacleRange;
		double viewAngle;
		vector<int> neighbours;
};


#endif //BOID_H
