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


// TODO: Suggestions from old code
// Suggestions for updates: 

// Bigger box and partial view following the boids (see DLA rendering code)
// Optimisation for better performance (the code has never been optimised)
// New classes that inherits the basic Boid but for other parameters
// --> Flies with 180 view angle and 0 alignement
// --> Try bird in V-shape ??
// Distinguish physical forces and behavioural forces 
// --> Two different vectors, with different caps
// --> Convert all behavioural forces to orientation forces or acceleration
// Ecosystem of boids with prey-predator relationships ??
// Reaction time for boids
// --> Implementation with a delay in the neighbour list updates
// --> This way, we can update the neighbour list at a slower rate than the
//     integration, thus saving computational power
// --> Two neighbour lists (physical and behavioural ones?),
//     the physical list can be associated with a fixed, smaller range
// Better wall avoidance (try seperation force in 1/r^2, in addition)
//                       (also the current behavioural one with rays is bad)
// Boid cohesion only with neighbours in sight (orientation force instead?)
// Remove orientation state variable --> member function instead for speed angle



////////////////////////////////////////////////////////////////////////////	    


#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>

#include "Math.h"
#include "Boid.h"
#include "Physics.h"
#include "Rendering.h"

using namespace std;


void info(Boid boid);


int main(int argc, char **argv)
{
	/////////////////////////////// Version ////////////////////////////////
	
	string versionCode = "1.4.0";
	
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--version")
	{
		cout << "Boids simulation, version " << versionCode << endl;
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
	
	
	/////////////////////////// Boid placement /////////////////////////////
	
	// try to use a specified number of boids if given in arguments
	int nBoids = 30;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--nBoids=")
	{
		string arg = argv[i];
		try {nBoids = stoi(arg.substr(9,arg.size()-9));}
		catch (...) {cout << "Error reading nBoids option: " << arg << endl;}
	}
	
	Boid boid(0,0,0,1e-3);
	vector<Boid> boids(nBoids,boid);
	world.placeBoids(boids);
	
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
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids");
	
	//window.setFramerateLimit(1);
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double t = 0;
	double dt = 0.01;
	int tSleep = dt*1000;
	
	bool pause = false;
	bool slowdown = false;
	bool accelerate = false;
	
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
				this_thread::sleep_for(std::chrono::milliseconds(10*tSleep));
			else if (accelerate)
				;// do not slow for rendering
			else
				this_thread::sleep_for(std::chrono::milliseconds(tSleep));
			
			////////////////////////// Time Step ///////////////////////////
			
			world.advanceTime(dt+1e-8,dt);
			t += dt;
			
			// TODO debug info
			Boid boid = world.getBoid(0);
			info(boid);
			
			/////////////////////////// Rendering //////////////////////////
			
			window.clear(sf::Color::White);
			world.render(window);
			window.display();
		}
		else
		{
			// do not use all cpu usage when in pause
			this_thread::sleep_for(std::chrono::milliseconds(100));
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


