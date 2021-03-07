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


// TODO: Encapsulate generation run as function of N variables to minimise
// TODO: Add penalty for bumping into walls ? How ?

// Idea: Variant where the target follows the boid (i.e. as a predator)
//       and the boid must avoid it.

// Better minimisation? - Use standard min algo? (i.e. gradient descent)
//                      - perturb one or few weights at a time?
//                      - remember direction of the improvement 
//                        (i.e. which weight) 


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
                    NNParams pnn, WorldParams pworld, int numGen, 
                    double simTime, double &bestScore);
void renderGenerationInfo(sf::RenderWindow &window, int numGen);
void renderBoidAndTarget(sf::RenderWindow &window, BoidNN boid, 
                         double scaleX, double scaleY);
void renderTargets(sf::RenderWindow &window, vector<BoidNN> boids, 
                         double scaleX, double scaleY);


int main(int argc, char **argv)
{
	// Version
	
	string versionCode = "1.0.0";
	if (detectOption(argc, argv, "version"))
	{
		cout << "Boids (seekers basic), version " << versionCode << endl;
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
		cout << "          initNNFile      string           file for nn initialisation   " << endl;
		cout << endl;
		cout << "Controls:   Press Space to pause the simulation   " << endl;
		cout << "            Press S to slow down the simulation   " << endl;
		cout << "            Press A to accelerate the simulation  " << endl;
		cout << "            Press M to run the simulation at max speed  " << endl;
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
	
	NeuralNetwork nnetwork(2,1,2,2,seed);
	
	string nninitfilename = ""; readOption(argc, argv, "initNNFile", nninitfilename);
	if (nninitfilename.empty()) nnetwork.initRandom();
	else nnetwork.loadFromFile(nninitfilename);
	
	string nnfilename = "nnetwork.dat";
	nnetwork.saveToFile(nnfilename);
	
	// Window
	
	int windowSizeX = 600; readOption(argc, argv, "windowSizeX", windowSizeX);
	int windowSizeY = 600; readOption(argc, argv, "windowSizeY", windowSizeY);
	
	sf::RenderWindow window(sf::VideoMode(windowSizeX,windowSizeY),"Boids - Seekers (basic)");
	window.setFramerateLimit(30);
	
	// Create neural network and other parameter structures
	
	double noiseStd = 0.05; readOption(argc, argv, "noiseStd", noiseStd);
	
	struct NNParams pnn = {0.0, noiseStd, nnfilename};
	struct WorldParams pworld = {seed, nBoids, boxSizeX, boxSizeY, avgWalls};
	struct WindowParams pwindow = {false, false, false, false};
	
	// Call simulation for first generation
	
	int numGen = 1;
	double simTime = 10; readOption(argc, argv, "simTime", simTime);
	double bestScore = -10.0;
	
	while (window.isOpen())
	{
		run_generation(window, pwindow, pnn, pworld, numGen, simTime, bestScore);
		numGen ++;
		pworld.seed = pworld.seed + 1;
	}
	
	return 0;
}




void run_generation(sf::RenderWindow &window, WindowParams &pwindow,
                    NNParams pnn, WorldParams pworld, int numGen, 
                    double simTime, double &bestScore)
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
		// "Annealing method": We adapt the noise intensity depending on
		// the score of the boid. If it is bad (very negative) we use a 
		// large value of the noise.
		
		double mean = 0.0;
		double std = pnn.noiseStd;
		if (bestScore < 0) std *= abs(bestScore);
		
		nnetworks[i].perturbWeights(mean, std);
		nnetworks[i].perturbBiases(mean, std);
		
		/////////////////////////////////////////
		// Restrict degrees of freedom (implementation requires intermediate
		// layer but these are unnecessary neurons). Therefore we directly
		// copy the value of the sensors in the intermediate layer.
		nnetworks[i].setWeight(0,0,0,1.0);
		nnetworks[i].setWeight(0,0,1,0.0);
		nnetworks[i].setWeight(0,1,0,0.0);
		nnetworks[i].setWeight(0,1,1,1.0);
		nnetworks[i].setBias(0,0,0.0);
		nnetworks[i].setBias(0,1,0.0);
		/////////////////////////////////////////
		
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
			
			x = R * cos(angl) + boids[i].getPosX();
			y = R * sin(angl) + boids[i].getPosY();
		}
		
		boids[i].setTarget(x,y);
	}
	
	////////////////////////////// Main Loop ///////////////////////////////
	
	double t = 0;
	double dt = 0.01;
	double timeConvFactorBase = 1;
	double timeConvFactor = timeConvFactorBase;
	
	system_clock::time_point lastFrame = system_clock::now();
	
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
				
				if (pwindow.maxspeed) window.setFramerateLimit(-1);
				else window.setFramerateLimit(30);
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
			
			system_clock::time_point presentFrame = system_clock::now();
			double time_real = duration_cast<duration<double>> 
			                   (presentFrame - lastFrame).count();
			
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
				for (int i=0; i<boids.size(); i++)
				{
					double time = boids[i].detectTarget(t_step,0.3);
					
					// if boid reached target, place a new target to catch
					// and increase score 
					if (time>0)
					{
						double x,y; x=y=-1;
						while (x<0 || y<0 || x>pworld.sizeX || y>pworld.sizeY)
						{
							double angl = 2*PI*dist01(gen);
							double R = min(pworld.sizeX/2.0,pworld.sizeY/2.0);
							
							x = R * cos(angl) + boids[i].getPosX();
							y = R * sin(angl) + boids[i].getPosY();
						}
						
						boids[i].setTarget(x,y);
						boids[i].rewardTargetCaught();
					}
				}
				// boid score update
				for (int i=0; i<boids.size(); i++) boids[i].updateScore(t_step);
			}
			
			lastFrame = system_clock::now();
			world.printProfilingData();
			
			/////////////////////////// Rendering //////////////////////////
			
			double scaleX = window.getSize().x/pworld.sizeX;
			double scaleY = window.getSize().y/pworld.sizeY;
			
			window.clear(sf::Color::White);
			world.render(window);
			//world.renderDebug(window,0,true);
			//renderTargets(window, boids, scaleX, scaleY);
			renderBoidAndTarget(window, boids[0], scaleX, scaleY);
			renderGenerationInfo(window, numGen);
			window.display();
		}
		else
		{
			// do not use all cpu usage when in pause
			this_thread::sleep_for(milliseconds(100));
			
			lastFrame = system_clock::now();
		}
	}
	
	/////////////////////// Selection of the fittest ///////////////////////
	
	// Detect fittest
	
	int fittest = 0;
	bestScore = boids[0].getScore();
	
	for (int i=0; i<boids.size(); i++)
	{
		double score = boids[i].getScore();
		
		if (score > bestScore)
		{
			fittest = i;
			bestScore = score;
		}
	}
	
	// Debug info
	
	cout << "----------------------------------------------------" << endl;
	cout << "Generation " << numGen << ": best score is " << bestScore << endl,
	
	cout << fixed << setprecision(1);
	
	cout << "All scores are ";
	for (int i=0; i<boids.size(); i++) cout << setw(6) << boids[i].getScore() << " ";
	cout << endl;
	
	cout << "All times are  ";
	for (int i=0; i<boids.size(); i++) cout << setw(6) << boids[i].detectTarget(0.0) << " ";
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


void renderTargets(sf::RenderWindow &window, vector<BoidNN> boids, double scaleX, double scaleY)
{
	sf::CircleShape circleTarget(1);
	circleTarget.setScale(0.5*scaleX*0.3,0.5*scaleY*0.3);
	circleTarget.setOrigin(1,1);
	circleTarget.setFillColor(sf::Color::Blue);

	for (int i=0; i<boids.size(); i++)
	{
		circleTarget.setPosition(sf::Vector2f(boids[i].getTargetX()*scaleX,boids[i].getTargetY()*scaleY));
		window.draw(circleTarget);
	}
}



