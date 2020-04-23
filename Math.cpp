////////////////////////////////////////////////////////////////////////////
//                                                                        //
//               Source file for the implementation of                    //
//                   some  mathematical functions                         //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
////////////////////////////////////////////////////////////////////////////




#include "Math.h"

using namespace std;



double min(double a, double b) {return (a<b)?a:b;}
double max(double a, double b) {return (a>b)?a:b;}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}



// angle of 2 relative to 1
double angle(double x1, double y1, double x2, double y2)
{
	double angle = 0;
	double dx = x2-x1;
	double dy = y2-y1;
	
	if (dx>0) angle = atan(dy/dx)*180/PI;
	else if (dx<0) angle = atan(dy/dx)*180/PI+180;
	else if (dx==0) 
	{
		if (dy>0) angle = 90;
		else angle = -90;
	}
	
	return angle;
}



// angle between 0 and 360
double angle360(double angle)
{
	while (angle<0) angle += 360;
	while (angle>=360) angle -= 360;
	
	return angle;
}



// angle difference given between -180 and 180
double angleDifference(double angle1, double angle2)
{
	double angleDiff = angle1-angle2;
	
	while (angleDiff<-180) angleDiff += 360;
	while (angleDiff>=180) angleDiff -= 360;
	
	return angleDiff;
}




double sigmoid(double x)
{
	return tanh(x);
}



