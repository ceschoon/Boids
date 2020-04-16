////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                          * Boid Simulation *                           //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
//    Developped under Ubuntu 18.04 with g++ 7.4.0 and sfml 2.4           //
//    Compile with $ g++ -O3 -o boids boids.cpp \                         //
//                   Boid.cpp Physics.cpp Math.cpp \                      //
//                   -lsfml-graphics -lsfml-window -lsfml-system          //
//                                                                        //
//    Usage: ./boids --<option name>=<option value>                       //
//    Execute "./boids --help" for more details on the program usage      //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



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
// Boid cohesion only with neighbours in sight (orientation force instead?)


// TODO (urgent fixes):



// File structure:

// boids.cpp                Simulation main file, options 
// Boid.h, ~.cpp            Boid classes and subclasses, AI and
//                          behavioural interaction
// Physics.h ~.cpp          Walls & Rays classes, physical interactions, 
//                          physical neigh. lists??
// Math.h, ~.cpp            Some mathematical functions
// Rendering.h              Rendering functions


////////////////////////////////////////////////////////////////////////////	    


#include <SFML/Graphics.hpp>
#include <iostream>
#include <thread>

#include "Math.h"
#include "Boid.h"
#include "Physics.h"
#include "Rendering.h"

using namespace std;





int main(int argc, char **argv)
{
	/////////////////////////////// Version ////////////////////////////////
	
	string versionCode = "1.3.0";
	
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
	
	///////////////////////////// Wall placement ///////////////////////////
	
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
	
	// construct bounding box
	
	Wall wallBorder0(0,0,0,boxSizeY);
	Wall wallBorder1(0,boxSizeY,boxSizeX,boxSizeY);
	Wall wallBorder2(boxSizeX,boxSizeY,boxSizeX,0);
	Wall wallBorder3(boxSizeX,0,0,0);
	vector<Wall> walls = {wallBorder0,wallBorder1,wallBorder2,wallBorder3};
	
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
	
	for (int i=0; i<numWalls; i++)
		addRandomWallOnSquareGrid(walls, boxSizeX, boxSizeY, gen);
	
	/////////////////////////// Boid placement /////////////////////////////
	
	// try to use a specified number of boids if given in arguments
	int nBoids = 30;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--nBoids=")
	{
		string arg = argv[i];
		try {nBoids = stoi(arg.substr(9,arg.size()-9));}
		catch (...) {cout << "Error reading nBoids option: " << arg << endl;}
	}
	
	Boid boid(0,0,0);
	vector<Boid> boids(nBoids,boid);
	
	// special boid
	//boids[0].a = 1;
	//boids[0].b = 1;
	//boids[0].s = 1.0/180*40;
	//boids[0].f = 200;
	//boids[0].c = 10;
	//boids[0].m = 50;
	//boids[0].range = 5;
	
	placeBoids(boids, boxSizeX, boxSizeY, gen);
	updateNeighbours(boids,walls);
	
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
	
	double scaleX = windowSizeX/boxSizeX;
	double scaleY = windowSizeY/boxSizeY;
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double t = 0;
	double dt = 0.01;
	int tSleep = dt*1000;
	
	bool pause = false;
	bool slowdown = false;
	bool accelerate = false;
	
	bool isMouseInWindow = false;
	double mouseX = 0;
	double mouseY = 0;
	
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
			
			if (event.type == sf::Event::MouseMoved)
			{
				mouseX = event.mouseMove.x/scaleX;
				mouseY = event.mouseMove.y/scaleY;
			}
			
			if (event.type == sf::Event::MouseEntered)
			{
				isMouseInWindow = true;
			}
			
			if (event.type == sf::Event::MouseLeft)
			{
				isMouseInWindow = false;
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
			
			resetForce(boids);
			drivingForce(boids);
			drag(boids);
			separation(boids);
			cohesion(boids);
			alignment(boids);
			avoidWalls(boids, walls);
			//if (isMouseInWindow) mouseWorshipping(boids,walls,mouseX,mouseY);
			//capForce(boids);
			
			vector<Boid> boidsOld = boids;
			advanceTime(boids,dt);
			collideWalls(boidsOld,boids,walls);
			orientationWithSpeed(boids);
			updateNeighbours(boids, walls);
			t += dt;
			
			/////////////////////////// Rendering //////////////////////////
			
			window.clear(sf::Color::White);
			renderBoidsAsTriangles(window,boids,scaleX,scaleY);
			//renderBoidsHighlight(window,boids,scaleX,scaleY,0);
			//renderBoidsAsPoints(window,boids,scaleX,scaleY);
			renderForces(window,boids,scaleX,scaleY);
			renderWalls(window,walls,scaleX,scaleY);
			//if (isMouseInWindow) renderMouse(window,mouseX,mouseY,scaleX,scaleY);
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