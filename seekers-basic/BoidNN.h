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
		
		doBoidRepulsion = false;
		
		// Initialisations of attributes
		
		reachedTarget_ = false;
		rewarded_ = false;
		chrono_ = 0.0;
		targetX_ = 0.0;
		targetY_ = 0.0;
		
		score_ = 0.0;
		wdist_ = 0.1;
		wtime_ = 0.1;
		wtarget_ = 10;
		
		sensors_ = vector<double>(2,0);
	}
	
	double getScore() {return score_;}
	double getTargetX() {return targetX_;}
	double getTargetY() {return targetY_;}
	
	void setTarget(double x, double y) 
	{
		reachedTarget_ = false;
		rewarded_ = false;
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
	
	void rewardTargetCaught()
	{
		if(!rewarded_) score_ += wtarget_;
		rewarded_ = true;
	}
	
	void updateScore(double dt)
	{
		// lower score if the boid is far from the target
		double dist = distance(x,y,targetX_,targetY_);
		if (!reachedTarget_) score_ -= dt * wdist_ * dist;
		
		// lower score if the boid takes time to reach the target
		if (!reachedTarget_) score_ -= dt * wtime_ * chrono_;
	}
	
	void setNNetwork(NeuralNetwork *nnetwork) {nnetwork_ = nnetwork;}
	
	virtual void computeSensorialInput(const vector<Boid> &boids, const vector<Wall> &walls)
	{
		// First sensor is a measure of how close the boid points to its 
		// target. We use the cosine of the angle it makes with the target.
		
		double angleWithTarget = angleDifference(angle(x,y,targetX_,targetY_),orientation());
		sensors_[0] = angleWithTarget/180.0;
		
		// Second sensors is the distance to the target, normalised with a
		// max value
		
		double distToTarget = distance(x,y,targetX_,targetY_);
		double distMax = 100.0;
		sensors_[1] = distToTarget/distMax;
		
		
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
	bool rewarded_;
	double chrono_;
	double targetX_;
	double targetY_;
	
	double score_;
	double wdist_; // weight for distance to target in score calculation
	double wtime_; // weight for time to reach target (chrono_)
	double wtarget_; // reward for catching a target
	
	vector<double> sensors_;
	
	NeuralNetwork *nnetwork_;
};


#endif
