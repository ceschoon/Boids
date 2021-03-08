////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                          * Boid Simulation *                           //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020, October 2020                            //
//                                                                        //
//    Developped under Ubuntu 18.04 with g++ 7.4.0 and sfml 2.4           //
//                                                                        //
//    Usage: ./boids --<option name>=<option value>                       //
//    Execute "./boids --help" for more details on the program usage      //
//                                                                        //
////////////////////////////////////////////////////////////////////////////


// Suggestions for updates: 

// Neural network for boid behaviour
// Bigger box and partial view following the boids (see DLA rendering code)
// Optimisation for better performance
// New classes that inherits the basic Boid but for other parameters
// --> Flies with 180 view angle and 0 alignement
// --> Try bird in V-shape ??
// Ecosystem of boids with prey-predator relationships ??
// Reaction time for boids
// --> Implementation with a delay in the neighbour list updates
// --> This way, we can update the neighbour list at a slower rate than the
//     integration, thus saving computational power
// --> Two neighbour lists (physical and behavioural ones?),
//     the physical list can be associated with a fixed, smaller range
// Better wall-boid physics (add repulsive force in 1/r^2)


////////////////////////////////////////////////////////////////////////////


#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include "Math.h"
#include "Boid.h"
#include "Physics.h"
#include "Rendering.h"
#include "myoptions.h"

using namespace std;
using namespace std::chrono;


void info(Boid boid);



class myBoid : public Boid
{
	public: 
	
	myBoid(double x_, double y_, double orientation_, double v_)
	: Boid(x_,y_,orientation_,v_)
	{
		b = 0.1;             // default = 0.1
		s = 5.0/180;         // default = 5.0/180
		w = 25.0/180;        // default = 25.0/180
		
		a = 1;               // default = 1
		c = 1;               // default = 1
		
		f = 10;              // default = 10
		f1 = 10;             // default = 10
		f2 = 50;             // default = 10 or 50
		
		viewRange = 10;      // default = 10
		obstacleRange = 5;   // default = 5
		viewAngle = 120;     // default = 120
	}
};




int main(int argc, char **argv)
{
	/////////////////////////////// Version ////////////////////////////////
	
	string versionCode = "1.4.3";
	if (detectOption(argc, argv, "version"))
	{
		cout << "Boids, version " << versionCode << endl;
		cout << endl;
		
		return 0;
	}
	
	
	//////////////////////////////// Help //////////////////////////////////
	
	if (detectOption(argc, argv, "help"))
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
		cout << "          simTime         double           simulation time for one gen  " << endl;
		cout << endl;
		cout << "Controls:   Press Space to pause the simulation   " << endl;
		cout << "            Press S to slow down the simulation   " << endl;
		cout << "            Press A to accelerate the simulation  " << endl;
		cout << "            Press M to run the simulation at max speed  " << endl;
		cout << endl;
		
		return 0;
	}
	
	
	////////////////////////////// Randomness //////////////////////////////
	
	random_device true_gen;
	int seed = true_gen(); readOption(argc, argv, "seed", seed);
	
	cout << "seed = " << seed << endl;
	default_random_engine gen(seed);
	
	
	///////////////////////////// World Creation ///////////////////////////
	
	double boxSizeX = 30; readOption(argc, argv, "boxSizeX", boxSizeX);
	double boxSizeY = 30; readOption(argc, argv, "boxSizeY", boxSizeY);
	
	// create world
	World world(boxSizeX,boxSizeY,seed);
	
	// try to use a specified average number walls if given in arguments
	double avgWalls = 4; readOption(argc, argv, "avgWalls", avgWalls);
	
	// randomly place walls
	
	poisson_distribution<int> dist(avgWalls);
	int numWalls = dist(gen);
	
	for (int i=0; i<numWalls; i++) world.addRandomWallOnSquareGrid();
	
	
	/////////////////////////// Boid placement /////////////////////////////
	
	// try to use a specified number of boids if given in arguments
	int nBoids = 30; readOption(argc, argv, "nBoids", nBoids);
	
	// construct boids and vector of pointers
	vector<Boid> boids(nBoids, Boid(0,0,0,1e-3));
	vector<Boid*> boids_ptr(nBoids, NULL);
	
	for (int i=0; i<nBoids; i++) boids_ptr[i] = &boids[i];
	world.placeBoids(boids_ptr);
	
	/////////////////////////////// Window /////////////////////////////////
	
	int windowSizeX = 600; readOption(argc, argv, "windowSizeX", windowSizeX);
	int windowSizeY = 600; readOption(argc, argv, "windowSizeY", windowSizeY);
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids");
	window.setFramerateLimit(30);
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double simTime = -1; readOption(argc, argv, "simTime", simTime);
	
	double t = 0;
	double dt = 0.01;
	double timeConvFactorBase = 1;
	double timeConvFactor = timeConvFactorBase;
	
	bool pause = false;
	bool slowdown = false;
	bool accelerate = false;
	bool maxspeed = false;
	
	system_clock::time_point lastFrame = system_clock::now();
	
	while (window.isOpen() && (t<simTime || simTime<0))
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
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::M)
			{
				maxspeed = !maxspeed;
				
				if (maxspeed) window.setFramerateLimit(-1);
				else window.setFramerateLimit(30);
			}
		}
		
		if (!pause) 
		{
			//////////////////////////// Timing ////////////////////////////
			
			// maxspeed: techically not max speed but we use a number large 
			// enough so that the cpu is fully used between each frame
			
			if (slowdown)        timeConvFactor = timeConvFactorBase/5;
			else if (accelerate) timeConvFactor = timeConvFactorBase*5;
			else if (maxspeed)   timeConvFactor = timeConvFactorBase*100;
			else timeConvFactor = timeConvFactorBase;
			
			
			////////////////////////// Time Step ///////////////////////////
			
			system_clock::time_point presentFrame = system_clock::now();
			double time_real = duration_cast<duration<double>> 
			                   (presentFrame - lastFrame).count();
			
			double time_world = time_real*timeConvFactor;
			
			world.advanceTime(time_world,dt);
			t += time_world;
			
			lastFrame = system_clock::now();
			
			//// debug info
			//Boid boid = world.getBoid(0);
			//info(boid);
			
			world.printProfilingData(argc, argv);
			
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
			
			lastFrame = system_clock::now();
		}
	}
	
	return 0;
}




void info(Boid boid)
{
	double v2 = boid.vx*boid.vx + boid.vy*boid.vy;
	double f2 = boid.fx*boid.fx + boid.fy*boid.fy;
	
	cout << "|v| = " << sqrt(v2) << " "
	     << "|f| = " << sqrt(f2) << " "
	     << endl;
}


