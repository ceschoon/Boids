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
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

#include "Math.h"
#include "BoidNN.h"
#include "Physics.h"
#include "Rendering.h"
#include "NeuralNetwork.h"

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


void run_generation(sf::RenderWindow &window, NNParams pnn, WorldParams pworld, int numGen);
void renderGenerationInfo(sf::RenderWindow &window, int numGen);


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
	
	// try to use a specified average number walls if given in arguments
	double avgWalls = 4;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,11)=="--avgWalls=")
	{
		string arg = argv[i];
		try {avgWalls = stod(arg.substr(11,arg.size()-11));}
		catch (...) {cout << "Error reading avgWalls option: " << arg << endl;}
	}
	
	
	//////////////////////////// Neural Network ////////////////////////////
	
	// Base NN
	
	NeuralNetwork nnetwork(6,2,5,2,seed);
	nnetwork.initRandom();
	
	string nnfilename = "nnetwork.dat";
	nnetwork.saveToFile(nnfilename);
	
	/////////////////////////// Boid placement /////////////////////////////
	
	// try to use a specified number of boids if given in arguments
	int nBoids = 10;
	for (int i=0; i<argc; i++) if (string(argv[i]).substr(0,9)=="--nBoids=")
	{
		string arg = argv[i];
		try {nBoids = stoi(arg.substr(9,arg.size()-9));}
		catch (...) {cout << "Error reading nBoids option: " << arg << endl;}
	}
	
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
	
	// create neural network and world parameter structures
	
	struct NNParams pnn = {0.0, 0.5, nnfilename};
	struct WorldParams pworld = {seed, nBoids, boxSizeX, boxSizeY, avgWalls};
	
	// call simulation for first generation
	
	int numGen = 1; 
	run_generation(window, pnn, pworld, numGen);
	
	return 0;
}




void run_generation(sf::RenderWindow &window, NNParams pnn, 
                    WorldParams pworld, int numGen)
{
	// Create world and add walls
	
	World world(pworld.sizeX,pworld.sizeY,pworld.seed);
	
	default_random_engine gen(pworld.seed);
	poisson_distribution<int> dist(pworld.avgWalls);
	int numWalls = dist(gen);
	
	for (int i=0; i<numWalls; i++) world.addRandomWallOnSquareGrid();
	
	// Construct boids and vector of pointers
	
	vector<NeuralNetwork> nnetworks(pworld.nBoids, NeuralNetwork(0,0,0,0,pworld.seed));
	vector<BoidNN> boids(pworld.nBoids, BoidNN(0,0,0,1e-3));
	vector<Boid*> boids_ptr(pworld.nBoids, NULL);
	
	for (int i=0; i<pworld.nBoids; i++) 
	{
		nnetworks[i].loadFromFile(pnn.filename);
		nnetworks[i].perturbWeights(pnn.noiseMean, pnn.noiseStd);
		nnetworks[i].perturbBiases(pnn.noiseMean, pnn.noiseStd);
		
		boids[i].setNNetwork(&nnetworks[i]);
		boids_ptr[i] = &boids[i];
	}
	
	world.placeBoids(boids_ptr);
	
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
			
			/////////////////////////// Rendering //////////////////////////
			
			
			window.clear(sf::Color::White);
			world.render(window);
			//world.renderDebug(window,0,true);
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




