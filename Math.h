////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Header file for the implementation of                    //
//                   some  mathematical functions                         //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



#ifndef MATH_H
#define MATH_H

#include <vector>
#include <random>
#include <cmath>

using namespace std;


double const PI = 3.14159265358979;


double min(double a, double b);
double max(double a, double b);


double distance(double x1, double y1, double x2, double y2);


// angle of 2 relative to 1
double angle(double x1, double y1, double x2, double y2);

// angle between 0 and 360
double angle360(double angle);

// angle difference given between -180 and 180
double angleDifference(double angle1, double angle2);


double sigmoid(double x);


#endif // MATH_H
