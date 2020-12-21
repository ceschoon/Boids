#ifndef BOIDNN_H
#define BOIDNN_H

#include <iostream>

#include "Boid.h"
#include "Physics.h"
#include "NeuralNetwork.h"
#include "Math.h"


using namespace std;


class BoidNN : public Boid
{
	public: 
	
	BoidNN(double x_, double y_, double orientation_, double v_)
	: Boid(x_,y_,orientation_,v_)
	{
		// Parameters from parent class
		
		b = 0.1;             // default = 0.1
		s = 1.0/180;         // default = 5.0/180
		w = 25.0/180;        // default = 25.0/180
		
		a = 1;               // default = 1
		c = 1;               // default = 1
		
		f = 10;              // default = 10
		f1 = 10;             // default = 10
		f2 = 50;             // default = 10 or 50
		
		viewRange = 10;      // default = 10
		obstacleRange = 5;   // default = 5
		viewAngle = 120;     // default = 120
		
		// Initialisations of attributes
		
		reachedTarget_ = false;
		chrono_ = 0.0;
		targetX_ = 0.0;
		targetY_ = 0.0;
		
		sensors_ = vector<double>(1,0);
	}
	
	double getTargetX() {return targetX_;}
	double getTargetY() {return targetY_;}
	
	void setTarget(double x, double y) 
	{
		reachedTarget_ = false;
		chrono_ = 0.0;
		targetX_ = x;
		targetY_ = y;
	}
	
	double detectTarget(double dt, double tolerance=0.1)
	{
		if (abs(x-targetX_)<tolerance && abs(y-targetY_)<tolerance) 
		{
			reachedTarget_ = true;
		}
		
		if (reachedTarget_) return chrono_;
		else
		{
			chrono_ += dt;
			return -1.0;
		}
	}
	
	void setNNetwork(NeuralNetwork *nnetwork) {nnetwork_ = nnetwork;}
	
	virtual void computeSensorialInput(const vector<Boid> &boids, const vector<Wall> &walls)
	{
		// First sensor is a measure of how close the boid points to its 
		// target. We use the cosine of the angle it makes with the target.
		
		double angleWithTarget = angleDifference(angle(x,y,targetX_,targetY_),orientation());
		//sensors_[0] = sin(angleWithTarget);
		sensors_[0] = angleWithTarget/180.0;
		
		// The next N sensors are the distance to the first intersection
		// between a wall and rays covering the field of view of the boid.
		/*
		int N=3;
		for (int i=0; i<N; i++)
		{
			// construct ray
			
			double raySpan = 2*viewAngle/N;
			double rayRelAngle = i*raySpan - viewAngle + raySpan/2;
			
			Ray ray(x,y,orientation()+rayRelAngle);
			
			// find closest intersection with wall
			
			double distMin = obstacleRange;
			for (Wall wall : walls)
			{
				bool exists;
				double xInt,yInt; intersection(ray,wall,xInt,yInt,exists);
				double distInt = distance(x,y,xInt,yInt);
				
				if (exists && distInt<distMin) distMin = distInt;
			}
			
			// assign sensor value from distance to intersection
			
			sensors_[i] = 2*distMin/obstacleRange - 1;
		}
		*/
	}
	
	virtual void computeBehaviouralForces(const vector<Boid> &boids, const vector<Wall> &walls)
	{
		// compute values of sensors
		
		computeSensorialInput(boids, walls);
		
		// compute neural network prediction
		
		vector<double> nnoutput = nnetwork_->eval(sensors_);
		
		double f1_ = f + f1*nnoutput[0];   // parallel component
		double f2_ = f2*nnoutput[1];       // perpendicular component
		
		// update the Boid forces in the fixed xy-frame
		
		fx += f1_*cos(orientation()/180*PI) - f2_*sin(orientation()/180*PI);
		fy += f1_*sin(orientation()/180*PI) + f2_*cos(orientation()/180*PI);
	}
	
	protected:
	
	bool reachedTarget_;
	double chrono_;
	double targetX_;
	double targetY_;
	
	vector<double> sensors_;
	
	NeuralNetwork *nnetwork_;
};


#endif
