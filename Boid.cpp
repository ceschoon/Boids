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
#include <cmath>
#include "Boid.h"

using namespace std;



Boid::Boid(double x_, double y_, double orientation_)
{
	x = x_;
	y = y_;
	vx = 0;
	vy = 0;
	fx = 0;
	fy = 0;
	orientation = orientation_;
	
	a = 20;              // default = 20
	b = 10;              // default = 10
	s = 1.0/180*50 ;     // default = 1.0/180*100
	f = 100;             // default = 100
	c = 10;              // default = 10
	m = 30;              // default = 30
	w = 1.0/180*100;     // default = 1.0/180*300
	
	range = 10;          // default = 10
	obstacleRange = 5;   // default = 5
	viewAngle = 120;     // default = 120
	neighbours = {};
}