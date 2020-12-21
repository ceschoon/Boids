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


// Note: The boids do not perform better than simply going forward.
//       I believe the networks do not improve because the boids randomly 
//       step on the target too frequently. I should make the evaluation
//       more difficult so that they can hardly pass it by chance.

//       Problem: With the walls it could happen than a target is in a
//       closed region that the boid cannot access. Using a test with 
//       multiple targets could make it impossible to pass or would once
//       again be passed by chance by the boids whose all targets were 
//       accessible.

//       Solution: Place targets iteratively on rays starting from the boid.

// TODO: First of all, try to train boid with 1 sensor (direction of target)
//       in a box without walls.

// TODO: Add sensor for distance to target
// TODO: Best evaluation: get multiple targets one after the other ? (max count wins)
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


struct WindowParams
{
	bool pause;
	bool slowdown;
	bool accelerate;
	bool maxspeed;
};


void run_generation(sf::RenderWindow &window, WindowParams &pwindow,
                    NNParams pnn, WorldParams pworld, int numGen, double simTime);
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
		cout << "            Press M to accelerate the simulation (max speed)  " << endl;
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
	double avgWalls = 0;  readOption(argc, argv, "avgWalls", avgWalls);
	
	// Base Neural Network
	
	NeuralNetwork nnetwork(1,1,1,2,seed);
	nnetwork.initRandom();
	nnetwork.loadFromFile("nnetwork_test.dat"); // TODO temp (make option)
	
	string nnfilename = "nnetwork.dat";
	nnetwork.saveToFile(nnfilename);
	
	// Window
	
	int windowSizeX = 600; readOption(argc, argv, "windowSizeX", windowSizeX);
	int windowSizeY = 600; readOption(argc, argv, "windowSizeY", windowSizeY);
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids - Avoiders");
	window.setFramerateLimit(30);
	
	// Create neural network and other parameter structures
	
	double noiseStd = 0.01; readOption(argc, argv, "noiseStd", noiseStd);
	
	struct NNParams pnn = {0.0, noiseStd, nnfilename};
	struct WorldParams pworld = {seed, nBoids, boxSizeX, boxSizeY, avgWalls};
	struct WindowParams pwindow = {false, false, false, false};
	
	// Call simulation for first generation
	
	int numGen = 1;
	double simTime = 30; readOption(argc, argv, "simTime", simTime);
	
	while (window.isOpen())
	{
		run_generation(window, pwindow, pnn, pworld, numGen, simTime);
		numGen ++;
		pworld.seed = pworld.seed + 1;
	}
	
	return 0;
}




void run_generation(sf::RenderWindow &window, WindowParams &pwindow,
                    NNParams pnn, WorldParams pworld, int numGen, double simTime)
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
		
		// TODO temporary
		//nnetworks[i].setWeight(0,0,0,1.0);
		//nnetworks[i].setBias(0,0,0.0);
		
		// TODO temporary
		//nnetworks[i].loadFromFile("nnetwork_test.dat");
		
		boids[i].setNNetwork(&nnetworks[i]);
	}
	
	// Assign random targets for boids
	// place the target on a circle centred on the boid
	
	uniform_real_distribution<double> dist01(0,1);
	
	for (int i=0; i<pworld.nBoids; i++) 
	{
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
				pwindow.pause = !pwindow.pause;
			}
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::S)
			{
				pwindow.slowdown = !pwindow.slowdown;
			}
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::A)
			{
				pwindow.accelerate = !pwindow.accelerate;
			}
			
			if (event.type == sf::Event::KeyReleased && 
				event.key.code == sf::Keyboard::M)
			{
				pwindow.maxspeed = !pwindow.maxspeed;
			}
		}
		
		if (!pwindow.pause) 
		{
			//////////////////////////// Timing ////////////////////////////
			
			// maxspeed: techically not max speed but we use a number large 
			// enough so that the cpu is fully used between each frame
			
			if (pwindow.slowdown)        timeConvFactor = timeConvFactorBase/5;
			else if (pwindow.accelerate) timeConvFactor = timeConvFactorBase*5;
			else if (pwindow.maxspeed)   timeConvFactor = timeConvFactorBase*100;
			else timeConvFactor = timeConvFactorBase;
			
			////////////////////////// Time Step ///////////////////////////
			
			steady_clock::time_point presentFrame = steady_clock::now();
			double time_real = duration_cast<duration<double>> 
			                   (presentFrame - lastFrame).count();
			lastFrame = presentFrame;
			
			double time_world = time_real*timeConvFactor;
			
			// cut the time integration in small batches of t_step so that
			// we can regularly and accurately do the target detection
			
			bool last_step = false;
			double t0 = t;
			double t_step = 0.08;
			
			while (!last_step)
			{
				// last step must be so that we don't exeed the max time
				if (t+t_step>t0+time_world) 
				{
					t_step = t0+time_world-t;
					last_step = true;
				}
				
				// world time integration
				world.advanceTime(t_step,dt);
				t += t_step;
				
				// boid target detection
				for (int i=0; i<boids.size(); i++) boids[i].detectTarget(t_step,0.3);
			}
			
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
	
	/*
	vector<double> w = nnetworks[fittest].getAllWeights();
	vector<double> b = nnetworks[fittest].getAllBiases();
	
	int Ns = nnetworks[fittest].getNumSensors();
	int Nl = nnetworks[fittest].getNumLayers();
	int Np = nnetworks[fittest].getNumNeuronsPerLayer();
	int No = nnetworks[fittest].getNumOutputs();
	
	cout << endl;
	cout << "Network weights (fittest):" << endl;
	
	cout << endl;
	for (int j=0; j<Ns; j++) // sensor-layer
	{
		for (int i=0; i<Np; i++)
		{
			double ww = nnetworks[fittest].getWeight(0,i,j);
			cout << scientific << setprecision(2) << setw(10) << ww;
		}
		cout << endl;
	}
	
	for (int k=0; k<Nl-1; k++) // layer-layer
	{
		cout << endl;
		for (int j=0; j<Np; j++)
		{
			for (int i=0; i<Np; i++)
			{
				double ww = nnetworks[fittest].getWeight(k+1,i,j);
				cout << scientific << setprecision(2) << setw(10) << ww;
			}
			cout << endl;
		}
	}
	
	cout << endl;
	for (int j=0; j<Np; j++) // layer-output
	{
		for (int i=0; i<No; i++)
		{
			double ww = nnetworks[fittest].getWeight(Nl,i,j);
			cout << scientific << setprecision(2) << setw(10) << ww;
		}
		cout << endl;
	}
	
	cout << endl;
	cout << "Network biases (fittest):" << endl;
	
	cout << endl;
	for (int i=0; i<Np; i++) // sensor-layer
	{
		double bb = nnetworks[fittest].getBias(0,i);
		cout << scientific << setprecision(2) << setw(10) << bb;
	}
	cout << endl;
	
	for (int k=0; k<Nl-1; k++) // layer-layer
	{
		cout << endl;
		for (int i=0; i<Np; i++)
		{
			double bb = nnetworks[fittest].getBias(k+1,i);
			cout << scientific << setprecision(2) << setw(10) << bb;
		}
		cout << endl;
	}
	
	cout << endl;
	for (int i=0; i<No; i++) // layer-output
	{
		double bb = nnetworks[fittest].getBias(Nl,i);
		cout << scientific << setprecision(2) << setw(10) << bb;
	}
	cout << endl;
	cout << endl;
	*/
	
	/*
	for (double ang = -1.0; ang<=1.0; ang+=0.2)
	{
		vector<double> sensors(1,ang);
		vector<double> outputs = nnetworks[fittest].eval(sensors);
		
		cout << "Test eval on {";
		for (double val : sensors) cout << setw(5) << val << " "; cout << "}: ";
		for (double val : outputs) cout << setw(5) << val << " "; cout << endl;
	}
	*/
	
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



