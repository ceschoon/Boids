////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                          * Boid Simulation *                           //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    November 2020                                                       //
//                                                                        //
//    Developped under Ubuntu 18.04 with g++ 7.4.0 and sfml 2.4           //
//                                                                        //
//    Usage: ./boids --<option name>=<option value>                       //
//    Execute "./boids --help" for more details on the program usage      //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include "Math.h"
#include "Boid.h"
#include "Physics.h"
#include "Rendering.h"
#include "NeuralNetwork.h"

using namespace std;
using namespace std::chrono;


void info(Boid boid);


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
		
		targetX_ = 0.0;
		targetY_ = 0.0;
		
		sensors_ = vector<double>(4,0);
	}
	
	void setTarget(double x, double y) {targetX_=x; targetY_=y;}
	
	void setNNetwork(NeuralNetwork *nnetwork) {nnetwork_ = nnetwork;}
	
	virtual void computeSensorialInput(const vector<Boid> &boids, const vector<Wall> &walls)
	{
		// First sensor is a measure of how close the boid points to its 
		// target. We use the cosine of the angle it makes with the target.
		
		double angleWithTarget = angle(x,y,targetX_,targetY_) - orientation();
		sensors_[0] = cos(angleWithTarget);
		
		// The next N sensors are the distance to the first intersection
		// between a wall and rays covering the field of view of the boid.
		
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
	
	double targetX_;
	double targetY_;
	
	vector<double> sensors_;
	
	NeuralNetwork *nnetwork_;
};




int main(int argc, char **argv)
{
	/////////////////////////////// Version ////////////////////////////////
	
	string versionCode = "1.0.0";
	
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--version")
	{
		cout << "Boids Avoiders, version " << versionCode << endl;
		cout << endl;
		
		return 0;
	}
	
	
	//////////////////////////////// Help //////////////////////////////////
	
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,6)=="--help")
	{
		cout << "Usage: ./boids --<option name>=<option value>" << endl;
		cout << endl;
		cout << "Options:  <option name>   <option value>   <description>                " << endl;
		cout << "          version         -                version of the program       " << endl;
		cout << "          help            -                help for usage               " << endl;
		cout << "          seed            int              seed for random generation   " << endl;
		cout << "          avgWalls        double           average number of walls      " << endl;
		cout << "          nBoids          int              number of boids              " << endl;
		cout << "          boxSizeX        double           size of the box              " << endl;
		cout << "          boxSizeY        double           size of the box              " << endl;
		cout << "          windowSizeX     int              size of the window           " << endl;
		cout << "          windowSizeY     int              size of the window           " << endl;
		cout << endl;
		cout << "Controls:   Press Space to pause the simulation   " << endl;
		cout << "            Press S to slow down the simulation   " << endl;
		cout << "            Press A to accelerate the simulation  " << endl;
		cout << endl;
		
		return 0;
	}
	
	
	////////////////////////////// Randomness //////////////////////////////
	
	random_device true_gen;
	int seed = true_gen();
	
	// try to use a specified seed if given in arguments
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,7)=="--seed=")
	{
		string arg = argv[i];
		try {seed = stoi(arg.substr(7,arg.size()-7));}
		catch (...) {cout << "Error reading seed option: " << arg << endl;}
	}
	
	cout << "seed = " << seed << endl;
	default_random_engine gen(seed);
	
	
	///////////////////////////// World Creation ///////////////////////////
	
	double boxSizeX = 30;
	double boxSizeY = 30;
	
	// try to use a specified box size if given in arguments
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,11)=="--boxSizeX=")
	{
		string arg = argv[i];
		try {boxSizeX = stod(arg.substr(11,arg.size()-11));}
		catch (...) {cout << "Error reading boxSizeX option: " << arg << endl;}
	}
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,11)=="--boxSizeY=")
	{
		string arg = argv[i];
		try {boxSizeY = stod(arg.substr(11,arg.size()-11));}
		catch (...) {cout << "Error reading boxSizeX option: " << arg << endl;}
	}
	
	// create world
	World world(30,30,seed);
	
	// try to use a specified average number walls if given in arguments
	double avgWalls = 4;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,11)=="--avgWalls=")
	{
		string arg = argv[i];
		try {avgWalls = stod(arg.substr(11,arg.size()-11));}
		catch (...) {cout << "Error reading avgWalls option: " << arg << endl;}
	}
	
	// randomly place walls
	
	poisson_distribution<int> dist(avgWalls);
	int numWalls = dist(gen);
	
	for (int i=0; i<numWalls; i++) world.addRandomWallOnSquareGrid();
	
	
	//////////////////////////// Neural Network ////////////////////////////
	
	NeuralNetwork nnetwork(6,2,5,2);
	nnetwork.initRandom();
	
	/////////////////////////// Boid placement /////////////////////////////
	
	// try to use a specified number of boids if given in arguments
	int nBoids = 1;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--nBoids=")
	{
		string arg = argv[i];
		try {nBoids = stoi(arg.substr(9,arg.size()-9));}
		catch (...) {cout << "Error reading nBoids option: " << arg << endl;}
	}
	
	// construct boids and vector of pointers
	vector<BoidNN> boids(nBoids, BoidNN(0,0,0,1e-3));
	vector<Boid*> boids_ptr(nBoids, NULL);
	
	for (int i=0; i<nBoids; i++) 
	{
		boids[i].setNNetwork(&nnetwork);
		boids_ptr[i] = &boids[i];
	}
	
	world.placeBoids(boids_ptr);
	
	////////////////////////// Experimentation /////////////////////////////
	
	// TODO: temporary section
	
	//boids[0].computeSensorialInput(boids,world.getWalls());
	
	/////////////////////////////// Window /////////////////////////////////
	
	int windowSizeX = 600;
	int windowSizeY = 600;
	
	// try to use a specified window size if given in arguments
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,14)=="--windowSizeX=")
	{
		string arg = argv[i];
		try {windowSizeX = stoi(arg.substr(14,arg.size()-14));}
		catch (...) {cout << "Error reading windowSizeX option: " << arg << endl;}
	}
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,14)=="--windowSizeY=")
	{
		string arg = argv[i];
		try {windowSizeY = stoi(arg.substr(14,arg.size()-14));}
		catch (...) {cout << "Error reading windowSizeY option: " << arg << endl;}
	}
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids - Avoiders");
	
	window.setFramerateLimit(30);
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double t = 0;
	double dt = 0.01;
	double timeConvFactorBase = 1;
	double timeConvFactor = timeConvFactorBase;
	
	bool pause = false;
	bool slowdown = false;
	bool accelerate = false;
	
	steady_clock::time_point lastFrame = steady_clock::now();
	
	while (window.isOpen())
	{
		///////////////////////// Event Handling ///////////////////////////
		
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
			
			if (event.type == sf::Event::KeyPressed && 
				event.key.code == sf::Keyboard::Space)
			{
				pause = !pause;
			}
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::S)
			{
				slowdown = !slowdown;
			}
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::A)
			{
				accelerate = !accelerate;
			}
		}
		
		if (!pause) 
		{
			//////////////////////////// Timing ////////////////////////////
			
			if (slowdown)
				timeConvFactor = timeConvFactorBase/5;
			else if (accelerate)
				timeConvFactor = timeConvFactorBase*5;
			else
				timeConvFactor = timeConvFactorBase;
			
			////////////////////////// Time Step ///////////////////////////
			
			steady_clock::time_point presentFrame = steady_clock::now();
			double time_real = duration_cast<duration<double>> 
			                   (presentFrame - lastFrame).count();
			lastFrame = presentFrame;
			
			double time_world = time_real*timeConvFactor;
			
			world.advanceTime(time_world,dt);
			t += time_world;
			
			//// debug info
			//vector<Boid> bboids = world.getBoids();
			//cout << "got nBoids = " << world.getNumBoids() << endl;
			//info(bboids[0]);
			//Boid boid = world.getBoid(0);
			//info(boid);
			
			/////////////////////////// Rendering //////////////////////////
			
			
			window.clear(sf::Color::White);
			world.render(window);
			//world.renderDebug(window,0,true);
			window.display();
			
		}
		else
		{
			// do not use all cpu usage when in pause
			this_thread::sleep_for(milliseconds(100));
			
			lastFrame = steady_clock::now();
		}
	}
	
	return 0;
}




void info(Boid boid)
{
	double v2 = boid.vx*boid.vx + boid.vy*boid.vy;
	double f2 = boid.fx*boid.fx + boid.fy*boid.fy;
	
	cout << "Info boid 0: "
	     << "x = " << boid.x << " "
	     << "y = " << boid.y << " "
	     << "|v| = " << sqrt(v2) << " "
	     << "|f| = " << sqrt(f2) << " "
	     << endl;
}


