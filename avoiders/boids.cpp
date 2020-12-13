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


// TODO: place the target away for the boid's initial position
//       or try to get multiple targets one after the other (max count)
// TODO: ? network statistics for monitoring ?
// TODO: perturb one or few weights at a time
// TODO: remember direction (i.e. which weight) of the improvement


#include <SFML/Graphics.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

#include "Math.h"
#include "BoidNN.h"
#include "Physics.h"
#include "Rendering.h"
#include "NeuralNetwork.h"
#include "myoptions.h"

using namespace std;
using namespace std::chrono;


struct NNParams
{
	double noiseMean;
	double noiseStd;
	
	string filename;
};


struct WorldParams
{
	int seed;
	int nBoids;
	
	double sizeX;
	double sizeY;
	double avgWalls;
};


void run_generation(sf::RenderWindow &window, NNParams pnn, WorldParams pworld, int numGen, double simTime);
void renderGenerationInfo(sf::RenderWindow &window, int numGen);
void renderBoidAndTarget(sf::RenderWindow &window, BoidNN boid, 
                         double scaleX, double scaleY);

int main(int argc, char **argv)
{
	// Version
	
	string versionCode = "1.0.0";
	if (detectOption(argc, argv, "version"))
	{
		cout << "Boids Avoiders, version " << versionCode << endl;
		cout << endl;
		
		return 0;
	}
	
	// Help
	
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
		cout << "          noiseStd        double           noise std on copies of nn    " << endl;
		cout << endl;
		cout << "Controls:   Press Space to pause the simulation   " << endl;
		cout << "            Press S to slow down the simulation   " << endl;
		cout << "            Press A to accelerate the simulation  " << endl;
		cout << endl;
		
		return 0;
	}
	
	// Random number generation
	
	random_device true_gen;
	int seed = true_gen();
	
	readOption(argc, argv, "seed", seed);
	cout << "seed = " << seed << endl;
	
	// World Creation
	
	int nBoids = 10; readOption(argc, argv, "nBoids", nBoids);
	
	double boxSizeX = 30; readOption(argc, argv, "boxSizeX", boxSizeX);
	double boxSizeY = 30; readOption(argc, argv, "boxSizeY", boxSizeY);
	double avgWalls = 4;  readOption(argc, argv, "avgWalls", avgWalls);
	
	// Base Neural Network
	
	NeuralNetwork nnetwork(6,2,5,2,seed);
	nnetwork.initRandom();
	
	string nnfilename = "nnetwork.dat";
	nnetwork.saveToFile(nnfilename);
	
	// Window
	
	int windowSizeX = 600; readOption(argc, argv, "windowSizeX", windowSizeX);
	int windowSizeY = 600; readOption(argc, argv, "windowSizeY", windowSizeY);
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids - Avoiders");
	window.setFramerateLimit(30);
	
	// Create neural network and world parameter structures
	
	double noiseStd = 0.05; readOption(argc, argv, "noiseStd", noiseStd);
	
	struct NNParams pnn = {0.0, noiseStd, nnfilename};
	struct WorldParams pworld = {seed, nBoids, boxSizeX, boxSizeY, avgWalls};
	
	// Call simulation for first generation
	
	int numGen = 1;
	double simTime = 60; readOption(argc, argv, "simTime", simTime);
	
	while (window.isOpen())
	{
		run_generation(window, pnn, pworld, numGen, simTime);
		numGen ++;
		pworld.seed = pworld.seed + 1;
	}
	
	return 0;
}




void run_generation(sf::RenderWindow &window, NNParams pnn, 
                    WorldParams pworld, int numGen, double simTime)
{
	//////////////////////////// Initialisation ////////////////////////////
	
	// Create world and add walls
	
	World world(pworld.sizeX,pworld.sizeY,pworld.seed);
	
	default_random_engine gen(pworld.seed);
	poisson_distribution<int> dist(pworld.avgWalls);
	int numWalls = dist(gen);
	
	for (int i=0; i<numWalls; i++) world.addRandomWallOnSquareGrid();
	
	// Construct boids and vector of pointers
	
	vector<BoidNN> boids(pworld.nBoids, BoidNN(0,0,0,1e-3));
	vector<Boid*> boids_ptr(pworld.nBoids, NULL);
	
	for (int i=0; i<pworld.nBoids; i++) boids_ptr[i] = &boids[i];
	world.placeBoids(boids_ptr);
	
	// Contruct neural networks for boids
	
	NeuralNetwork nnbase(0,0,0,0,pworld.seed);
	nnbase.loadFromFile(pnn.filename);
	vector<NeuralNetwork> nnetworks(pworld.nBoids, nnbase);
	
	for (int i=0; i<pworld.nBoids; i++) 
	{
		nnetworks[i].perturbWeights(pnn.noiseMean, pnn.noiseStd);
		nnetworks[i].perturbBiases(pnn.noiseMean, pnn.noiseStd);
		
		boids[i].setNNetwork(&nnetworks[i]);
	}
	
	// Assign random targets for boids
	// place the target on a circle centred on the boid
	
	uniform_real_distribution<double> dist01(0,1);
	
	for (int i=0; i<pworld.nBoids; i++) 
	{
		//double x = dist01(gen)*pworld.sizeX;
		//double y = dist01(gen)*pworld.sizeY;
		
		double x,y; x=y=-1;
		while (x<0 || y<0 || x>pworld.sizeX || y>pworld.sizeY)
		{
			double angl = 2*PI*dist01(gen);
			double R = min(pworld.sizeX/2.0,pworld.sizeY/2.0);
			
			x = R * cos(angl);
			y = R * sin(angl);
		}
		
		boids[i].setTarget(x,y);
	}
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double t = 0;
	double dt = 0.01;
	double timeConvFactorBase = 1;
	double timeConvFactor = timeConvFactorBase;
	
	bool pause = false;
	bool slowdown = false;
	bool accelerate = false;
	
	steady_clock::time_point lastFrame = steady_clock::now();
	
	while (window.isOpen() && t<simTime)
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
			
			// boid target detection and increment chronometer
			// TODO: should do the detection at each step, not just after
			//       the time evolution of world class. 
			//       But is it really a problem?
			for (int i=0; i<boids.size(); i++) boids[i].detectTarget(time_world,0.3);
			
			/////////////////////////// Rendering //////////////////////////
			
			double scaleX = window.getSize().x/pworld.sizeX;
			double scaleY = window.getSize().y/pworld.sizeY;
			
			window.clear(sf::Color::White);
			world.render(window);
			//world.renderDebug(window,0,true);
			renderBoidAndTarget(window, boids[0], scaleX, scaleY);
			renderGenerationInfo(window, numGen);
			window.display();
			
		}
		else
		{
			// do not use all cpu usage when in pause
			this_thread::sleep_for(milliseconds(100));
			
			lastFrame = steady_clock::now();
		}
	}
	
	/////////////////////// Selection of the fittest ///////////////////////
	
	// Detect fittest
	
	int fittest = 0;
	double bestTime = boids[0].detectTarget(0.0);
	
	for (int i=0; i<boids.size(); i++)
	{
		double time = boids[i].detectTarget(0.0);
		
		if (bestTime<0 || (time<bestTime && time>0))
		{
			fittest = i;
			bestTime = boids[i].detectTarget(0.0);
		}
	}
	
	// Debug info
	
	cout << "----------------------------------------------------" << endl;
	cout << "Generation " << numGen << ": best time is " << bestTime << endl,
	
	cout << "All times are ";
	for (int i=0; i<boids.size(); i++) cout << boids[i].detectTarget(0.0) << " ";
	cout << endl;
	
	// Save best neural network
	
	nnetworks[fittest].saveToFile(pnn.filename);
}




void renderGenerationInfo(sf::RenderWindow &window, int numGen)
{
	sf::Text text;
	stringstream sstr; sstr << "Gen " << numGen << endl;
	
	sf::Font font; font.loadFromFile("resources/fonts/NotoSans-Bold.ttf");
	
	text.setFont(font);
	text.setString(sstr.str());
	text.setCharacterSize(20);
	text.setFillColor(sf::Color::Red);
	text.setPosition(sf::Vector2f(5,0));
	
	window.draw(text);
}


void renderBoidAndTarget(sf::RenderWindow &window, BoidNN boid, 
                         double scaleX, double scaleY)
{
	sf::CircleShape triangle(1,3);
	triangle.setScale(0.5*scaleX*0.3,1*scaleY*0.3);
	triangle.setOrigin(1,1);
	triangle.setFillColor(sf::Color::Red);
	
	sf::CircleShape circleTarget(1);
	circleTarget.setScale(0.5*scaleX*0.3,0.5*scaleY*0.3);
	circleTarget.setOrigin(1,1);
	circleTarget.setFillColor(sf::Color::Red);
	
	triangle.setPosition(sf::Vector2f(boid.x*scaleX,boid.y*scaleY));
	triangle.setRotation(boid.orientation()+90);
	window.draw(triangle);
	
	circleTarget.setPosition(sf::Vector2f(boid.getTargetX()*scaleX,boid.getTargetY()*scaleY));
	window.draw(circleTarget);
}



