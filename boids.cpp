////////////////////////////////////////////////////////////////////////////
//                                                                        //
//                          * Boid Simulation *                           //
//                                                                        //
//    Author: Cédric Schoonen <cedric.schoonen1@gmail.com>                //
//    September 2019, April 2020                                          //
//                                                                        //
//    Developped under Ubuntu 18.04 with g++ 7.4.0 and sfml 2.4           //
//    Compile with $ g++ -O3 -o boids boids.cpp \                         //
//                   -lsfml-graphics -lsfml-window -lsfml-system          //
//                                                                        //
//    Usage: ./boids --<option name>=<option value>                       //
//    Execute "./boids --help" for more details on the program usage      //
//                                                                        //
////////////////////////////////////////////////////////////////////////////



// Suggestions for updates: 

// Bigger box and partial view following the boids (see DLA rendering code)
// Optimisation for better performance (the code has never been optimised)
// Seperate files as the code becomes a bit large
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



// File structure (to come):

// boids.cpp                Simulation main file, options 
// Boid.h, ~.cpp            Boid classes and subclasses, AI and
//                          behavioural interaction
// Physics.h ~.cpp          Walls & Rays classes, physical interactions, 
//                          physical neigh. lists??
// Rendering.h, ~.cpp       Rendering functions


////////////////////////////////////////////////////////////////////////////	    


#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <thread>
#include <random>

using namespace std;

double const PI = 3.14159265358979;
uniform_real_distribution<double> dist01(0,1);


////////////////////////// Boid class and methods //////////////////////////

class Boid
{
	public: 
		Boid(double x, double y, double orientation);
		
		double x,y;
		double vx,vy;
		double fx,fy;	// mass=1 (force=acceleration)
		double orientation;
		
		double a;	// seperation force coefficent: F_seperation = a/r^2			(towards target)
		double b;	// cohesion force coefficient:    F_cohesion = -b*r				(towards from target)
		double s;	// alignment coefficient:		 F_alignment = s*angleDiff^2	(perpendicular to the movement)
		double f;	// driving force:                  F_driving = f				(along the movement)
		double c;	// drag coefficient:                  F_drag = -1/2*c*v^2		(along the movement)
		double m;	// mouse affinity					 F_mouse = m				(towards mouse)
		double w;	// wall avoidance coefficient:        F_wall = w*angleDiff^2	(perpendicular to the movement)
		
		double range;
		double obstacleRange;
		double viewAngle;
		vector<int> neighbours;
};

Boid::Boid(double x_, double y_, double orientation_)
{
	x = x_;
	y = y_;
	vx = 0;
	vy = 0;
	fx = 0;
	fy = 0;
	orientation = orientation_;
	
	a = 20;              // default = 20
	b = 10;              // default = 10
	s = 1.0/180*100;     // default = 1.0/180*100
	f = 100;             // default = 100
	c = 10;              // default = 10
	m = 30;              // default = 30
	w = 1.0/180*300;     // default = 1.0/180*300
	
	range = 10;          // default = 10
	obstacleRange = 5;   // default = 5
	viewAngle = 120;     // default = 120
	neighbours = {};
}

void placeBoids(vector<Boid> &boids, double boxSizeX, double boxSizeY,
                default_random_engine &gen)
{
	double vInit = 0.00001;
	
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x = boxSizeX*dist01(gen);
		boids[i].y = boxSizeY*dist01(gen);
		boids[i].orientation = 360*dist01(gen);
		boids[i].vx = vInit*(-0.5+dist01(gen));	// to avoid problems when superposed
		boids[i].vy = vInit*(-0.5+dist01(gen)); // and to initiate randommly-orientated movement
	}
}

/////////////////////////// Math functions /////////////////////////////////

double min(double a, double b) {return (a<b)?a:b;}
double max(double a, double b) {return (a>b)?a:b;}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// angle of 2 relative to 1
double angle(double x1, double y1, double x2, double y2)
{
	double angle = 0;
	double dx = x2-x1;
	double dy = y2-y1;
	
	if (dx>0) angle = atan(dy/dx)*180/PI;
	else if (dx<0) angle = atan(dy/dx)*180/PI+180;
	else if (dx==0) 
	{
		if (dy>0) angle = 90;
		else angle = -90;
	}
	
	return angle;
}

// angle between 0 and 360
double angle360(double angle)
{
	while (angle<0) angle += 360;
	while (angle>=360) angle -= 360;
	
	return angle;
}

// angle difference given between -180 and 180
double angleDifference(double angle1, double angle2)
{
	double angleDiff = angle1-angle2;
	
	while (angleDiff<-180) angleDiff += 360;
	while (angleDiff>=180) angleDiff -= 360;
	
	return angleDiff;
}

////////////////////////// Wall and Ray classes ////////////////////////////

class Wall
{
	public:
		Wall(double x1, double y1, double x2, double y2);
		
		double x1;
		double y1;
		double x2;
		double y2;
};

Wall::Wall(double x1_, double y1_, double x2_, double y2_)
{
	x1 = x1_;
	y1 = y1_;
	x2 = x2_;
	y2 = y2_;
}

class Ray
{
	public:
		Ray(double originX, double originY, double angle);
		
		double originX;
		double originY;
		double angle;
};

Ray::Ray(double originX_, double originY_, double angle_)
{
	originX = originX_;
	originY = originY_;
	angle = angle_;
}

void intersection(Ray ray, Wall wall, double &xInt, double &yInt, bool &exists)
{
	exists = true;
	
	double x1 = wall.x1;
	double y1 = wall.y1;
	double x2 = wall.x2;
	double y2 = wall.y2;
	double x3 = ray.originX;
	double y3 = ray.originY;
	double x4 = x3 + cos(ray.angle*PI/180); // whatever on the ray line
	double y4 = y3 + sin(ray.angle*PI/180); // whatever on the ray line
	
	if (x1==x2 && x3==x4)
	{
		exists = false;
	}
	else if (x1==x2 && x3!=x4)
	{
		double slope34 = (y4-y3)/(x4-x3);
		
		xInt = x1;
		yInt = y3 + slope34*(xInt-x3);
		
		// check if intersection is on segment 12
		if (yInt<min(y1,y2) || yInt>max(y1,y2)) exists = false;
		
		// check if intersection is in the direction of the ray
		// if ray goes to positive x, intersection must be > x3
		if (cos(ray.angle*PI/180)>0 && xInt<x3) exists = false;
		// if ray goes to negative x, intersection must be < x3
		if (cos(ray.angle*PI/180)<0 && xInt>x3) exists = false;
		// if ray goes to positive y, intersection must be > y3
		if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
	}
	else if (x1!=x2 && x3==x4)
	{
		double slope12 = (y2-y1)/(x2-x1);
		
		xInt = x3;
		yInt = y1 + 1/slope12*(xInt-x1);
		
		// check if intersection is on segment 12
		if (xInt<min(x1,x2) || xInt>max(x1,x2)) exists = false;
		
		// check if intersection is in the direction of the ray
		// if ray goes to positive y, intersection must be > y3
		if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
		// if ray goes to negative y, intersection must be < y3
		if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
	}
	else
	{
		double slope12 = (y2-y1)/(x2-x1);
		double slope34 = (y4-y3)/(x4-x3);
		
		if (slope12==slope34) exists = false; // parallel
		else
		{
			xInt = (y1-y3-slope12*x1+slope34*x3) / (slope34-slope12);
			yInt = y1 + slope12 * (xInt-x1);
			
			// check if intersection is on segment 12
			if (xInt<min(x1,x2) || xInt>max(x1,x2)) exists = false;
			
			// check if intersection is in the direction of the ray
			// if ray goes to positive x, intersection must be > x3
			if (cos(ray.angle*PI/180)>0 && xInt<x3) exists = false;
			// if ray goes to negative x, intersection must be < x3
			if (cos(ray.angle*PI/180)<0 && xInt>x3) exists = false;
			// if ray goes to positive y, intersection must be > y3
			if (sin(ray.angle*PI/180)>0 && yInt<y3) exists = false;
			// if ray goes to negative y, intersection must be < y3
			if (sin(ray.angle*PI/180)<0 && yInt>y3) exists = false;
		}
	}
}

/////////////////////// Neighbour tests and updates ////////////////////////

bool isNeighbour(int index, Boid boid)
{
	bool isIt = false;
	vector<int> neighbours = boid.neighbours;
	
	for (int i=0; i<neighbours.size(); i++)
	{
		if (index == neighbours[i]) 
		{
			isIt = true;
			break;
		}
	}
	
	return isIt;
}

void updateNeighbours(vector<Boid> &boids, vector<Wall> walls)
{
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = {};
		
		for (int j=0; j<boids.size(); j++)
		{
			if (i!=j)
			{
				// distance condition
				double d = distance(boids[i].x,boids[i].y,boids[j].x,boids[j].y);
				
				// visibility condition (angle)
				double angleij = angle(boids[i].x,boids[i].y,boids[j].x,boids[j].y);
				
				// visibility condition (no wall)
				bool viewObstructed = false;
				Ray ray(boids[i].x,boids[i].y,angleij);
				
				for (int k=0; k<walls.size(); k++)
				{
					bool exists;
					double xInt,yInt;
					intersection(ray,walls[k],xInt,yInt,exists);
					if (exists && distance(boids[i].x,boids[i].y,xInt,yInt)
						<distance(boids[i].x,boids[i].y,boids[j].x,boids[j].y)) 
					{
						viewObstructed = true;
						break;
					}
				}
				
				if (d < boids[i].range && 
					abs(angleDifference(angleij,boids[i].orientation))<boids[i].viewAngle &&
					!viewObstructed)
				{
					neighbours.push_back(j);
				}
			}
		}
		
		boids[i].neighbours = neighbours;
	}
}

/////////////////////////////// Rendering //////////////////////////////////

void renderBoidsAsTriangles(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::CircleShape triangle(1,3);
    triangle.setScale(0.5*scaleX*0.3,1*scaleY*0.3);
    triangle.setOrigin(1,1);

    for (int i=0; i<boids.size(); i++)
    {
        triangle.setFillColor(sf::Color::Blue);
        triangle.setPosition(sf::Vector2f(boids[i].x*scaleX,boids[i].y*scaleY));
        triangle.setRotation(boids[i].orientation+90);
        window.draw(triangle);
    }
}

void renderBoidsHighlight(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY, int indexToHighlight)
{
    sf::CircleShape triangle(1,3);
    triangle.setScale(0.5*scaleX*0.3,1*scaleY*0.3);
    triangle.setOrigin(1,1);

    for (int i=0; i<boids.size(); i++)
    {
    	if (i==indexToHighlight) triangle.setFillColor(sf::Color::Red);
    	else if (isNeighbour(i,boids[indexToHighlight])) triangle.setFillColor(sf::Color::Green);
        else triangle.setFillColor(sf::Color::Blue);
        
        triangle.setPosition(sf::Vector2f(boids[i].x*scaleX,boids[i].y*scaleY));
        triangle.setRotation(boids[i].orientation+90);
        window.draw(triangle);
    }
}

void renderBoidsAsPoints(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::CircleShape circle(0.1);
    circle.setScale(scaleX,scaleY);
    circle.setOrigin(0.1,0.1);
    circle.setFillColor(sf::Color::Blue);

    for (int i=0; i<boids.size(); i++)
    {
        circle.setPosition(sf::Vector2f(
        	boids[i].x*scaleX,
        	boids[i].y*scaleY
        	));
        window.draw(circle);
    }
}

void renderForces(sf::RenderWindow &window, vector<Boid> boids, double scaleX, double scaleY)
{
    sf::RectangleShape line(sf::Vector2f(1, 0.03));
    line.setFillColor(sf::Color::Black);

    for (int i=0; i<boids.size(); i++)
    {
    	double F = sqrt(boids[i].fx*boids[i].fx+boids[i].fy*boids[i].fy);
    	double angleF = angle(0,0,boids[i].fx,boids[i].fy);
    	line.setScale(scaleX*F/4,scaleY);
        line.setPosition(sf::Vector2f(
        	boids[i].x*scaleX,
        	boids[i].y*scaleY
        	));
        line.setRotation(angleF);
        window.draw(line);
    }
}

void renderWalls(sf::RenderWindow &window, vector<Wall> walls, double scaleX, double scaleY)
{
    sf::RectangleShape line(sf::Vector2f(1, 0.05));
	line.setOrigin(0,0.025);
    line.setFillColor(sf::Color::Black);

    for (int i=0; i<walls.size(); i++)
    {
    	double x1 = walls[i].x1;
    	double y1 = walls[i].y1;
    	double x2 = walls[i].x2;
    	double y2 = walls[i].y2;
    	
    	double L = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    	double angleL = angle(x1,y1,x2,y2);
    	line.setScale(scaleX*L,scaleY);
        line.setPosition(sf::Vector2f(x1*scaleX, y1*scaleY));
        line.setRotation(angleL);
        window.draw(line);
    }
}

void renderMouse(sf::RenderWindow &window, double mouseX, double mouseY, double scaleX, double scaleY)
{
    sf::CircleShape circle(0.1);
    circle.setScale(scaleX,scaleY);
    circle.setOrigin(0.1,0.1);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(sf::Vector2f(mouseX*scaleX, mouseY*scaleY));
    window.draw(circle);
}

//////////////////////////////// Forces ////////////////////////////////////

void resetForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].fx = 0;
		boids[i].fy = 0;
	}
}

void drivingForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].fx += boids[i].f*cos(boids[i].orientation*PI/180);
		boids[i].fy += boids[i].f*sin(boids[i].orientation*PI/180);
	}
}

void drag(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		double speed2 = boids[i].vx*boids[i].vx+boids[i].vy*boids[i].vy;
		double angleSpeed = angle(0,0,boids[i].vx,boids[i].vy);
		
		boids[i].fx -= 0.5*boids[i].c*speed2*cos(angleSpeed*PI/180);
		boids[i].fy -= 0.5*boids[i].c*speed2*sin(angleSpeed*PI/180);
	}
}

void separation(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		
		double x1 = boids[i].x;
		double y1 = boids[i].y;
			
		for (int j=0; j<neighbours.size(); j++)
		{
			double x2 = boids[neighbours[j]].x;
			double y2 = boids[neighbours[j]].y;
			double r = distance(x1,y1,x2,y2);
			double angle12 = angle(x2,y2,x1,y1);
			
			// no test for r<range as it is done in the neighbour list
			double F = boids[i].a/(r*r);
			boids[i].fx += F*cos(angle12*PI/180);
			boids[i].fy += F*sin(angle12*PI/180);	
		}
	}
}

void cohesion(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		if (neighbours.size() == 0) continue; 
		
		double x1 = boids[i].x;
		double y1 = boids[i].y;
		
		// gravity center of neighbour boids
		double x2 = 0;
		double y2 = 0;
		
		for (int j=0; j<neighbours.size(); j++)
		{
			double x3 = boids[neighbours[j]].x;
			double y3 = boids[neighbours[j]].y;
			
			x2 += x3/neighbours.size();
			y2 += y3/neighbours.size();
		}
		
		double r = distance(x1,y1,x2,y2);
		double angle12 = angle(x2,y2,x1,y1);
		
		// no test for r<range as it is done in the neighbour list
		double F = -boids[i].b*r;
		
		boids[i].fx += F*cos(angle12*PI/180);
		boids[i].fy += F*sin(angle12*PI/180);
	}
}

void alignment(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		int N = neighbours.size();
		double orientation1 = boids[i].orientation;
		double avgOrientation = 0;
		
		if (N>0)
		{
			for (int j=0; j<neighbours.size(); j++)
			{
				avgOrientation += angle360(boids[neighbours[j]].orientation)/N;
			}
			
			double angleDiff = angleDifference(avgOrientation,orientation1);
			double F = boids[i].s*angleDiff;
			
			boids[i].fx += F*cos(orientation1*PI/180+PI/2);
			boids[i].fy += F*sin(orientation1*PI/180+PI/2);
		}
	}
}

void avoidWalls(vector<Boid> &boids, vector<Wall> walls)
{	
	for (int i=0; i<boids.size(); i++)
	{
		vector<int> neighbours = boids[i].neighbours;
		double orientation = boids[i].orientation;
			
		bool foundFreeRay = false;
		bool outOfRays = false;
		Ray ray(boids[i].x, boids[i].y, boids[i].orientation);
		
		while (foundFreeRay == false && outOfRays == false)
		{	 
			foundFreeRay = true;
			for (int j=0; j<walls.size(); j++)
			{
				// check if ray intersect the wall
				double xInt = 0;
				double yInt = 0;
				bool exists = false;
				intersection(ray, walls[j], xInt, yInt, exists);
				bool obstructed = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				
				// check if neighbour rays also intersect the wall
				double delta = 15;
				Ray rayLeft(ray.originX, ray.originY, ray.angle-delta);
				intersection(rayLeft, walls[j], xInt, yInt, exists);
				bool obstructedLeft = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				Ray rayRight(ray.originX, ray.originY, ray.angle+delta);
				intersection(rayRight, walls[j], xInt, yInt, exists);
				bool obstructedRight = exists && distance(ray.originX,ray.originY,xInt,yInt)<boids[i].obstacleRange;
				
				if (obstructed || obstructedLeft || obstructedRight)
				{
					foundFreeRay = false;
					break;
				}	
			}
			
			if (foundFreeRay == false)
			{
				// search at a new angle
				double angleStep = 3;
				if (angleDifference(ray.angle,orientation) >= 0) 
					ray.angle = orientation - (ray.angle-orientation) - angleStep;
				else if (angleDifference(ray.angle,orientation) < 0)  
					ray.angle = orientation - (ray.angle-orientation) + angleStep;
			}
			
			if (abs(angleDifference(ray.angle,orientation)) > boids[i].viewAngle)
				outOfRays = true;
		}
		
		if (foundFreeRay == true) 
		{
			double angleDiff = angleDifference(ray.angle,orientation);
			double F = boids[i].w*angleDiff;
			
			boids[i].fx += F*cos(orientation*PI/180+PI/2);
			boids[i].fy += F*sin(orientation*PI/180+PI/2);
		}
		else 
		{
			boids[i].orientation += 180;
		}
	}
}

void mouseWorshipping(vector<Boid> &boids, vector<Wall> walls, double mouseX, double mouseY)
{
	for (int i=0; i<boids.size(); i++)
	{			
		// distance condition
		double d = distance(boids[i].x,boids[i].y,mouseX,mouseY);
		
		// visibility condition (angle)
		double angleMouse = angle(boids[i].x,boids[i].y,mouseX,mouseY);
		
		// visibility condition (no wall)
		bool viewObstructed = false;
		Ray ray(boids[i].x,boids[i].y,angleMouse);
		
		for (int k=0; k<walls.size(); k++)
		{
			bool exists;
			double xInt,yInt;
			intersection(ray,walls[k],xInt,yInt,exists);
			if (exists && distance(boids[i].x,boids[i].y,xInt,yInt)
				<distance(boids[i].x,boids[i].y,mouseX,mouseY)) 
			{
				viewObstructed = true;
				break;
			}
		}
		
		if (d < boids[i].range && 
			abs(angleDifference(angleMouse,boids[i].orientation))<boids[i].viewAngle &&
			!viewObstructed)
		{
			boids[i].fx += boids[i].m*cos(angleMouse*PI/180);
			boids[i].fy += boids[i].m*sin(angleMouse*PI/180);
		}
	}
}

void capForce(vector<Boid> &boids)
{
	for (int i=0; i<boids.size(); i++)
	{
		double F = sqrt(boids[i].fx*boids[i].fx+boids[i].fy*boids[i].fy);
		double angleF = angle(0,0,boids[i].fx,boids[i].fy);
		
		if (F>100) 
		{
			boids[i].fx = 100*cos(angleF*PI/180);
			boids[i].fy = 100*sin(angleF*PI/180);
		}
	}
}

////////////////////// Step forward - related functions ////////////////////

void advanceTime(vector<Boid> &boids, double dt)
{
	for (int i=0; i<boids.size(); i++)
	{
		boids[i].x += boids[i].vx*dt + 0.5*boids[i].fx*dt*dt;
		boids[i].y += boids[i].vy*dt + 0.5*boids[i].fy*dt*dt;
		
		boids[i].vx += boids[i].fx*dt;
		boids[i].vy += boids[i].fy*dt;
	}
}

void orientationWithSpeed(vector<Boid> &boids)
{	
	for (int i=0; i<boids.size(); i++)
	{
		double orientation1 = boids[i].orientation;
		double angleSpeed = angle(0,0,boids[i].vx,boids[i].vy);
		boids[i].orientation = angleSpeed;
	}
}

void collideWalls(vector<Boid> &boidsOld, vector<Boid> &boidsNew, vector<Wall> walls)
{
	for (int i=0; i<boidsNew.size(); i++)
	{		
		for (int j=0; j<walls.size(); j++)
		{
			double x1 = walls[j].x1;
			double y1 = walls[j].y1;
			double x2 = walls[j].x2;
			double y2 = walls[j].y2;
			double x3 = boidsOld[i].x;
			double y3 = boidsOld[i].y;
			double x4 = boidsNew[i].x;
			double y4 = boidsNew[i].y;
			
			// Use Ray-Wall intersection code to detect collision
			
			double angle34 = angle(x3,y3,x4,y4);
			Ray ray(x3,y3,angle34);
			
			bool exists;
			double xInt, yInt;
			intersection(ray,walls[j],xInt,yInt,exists);
			
			double d34 = distance(x3,y3,x4,y4);
			double d3Int = distance(x3,y3,xInt,yInt);
			bool isCollision = exists && d3Int<d34;
			
			if (isCollision)
			{
				// mirror velocities
				
				double vx = (boidsOld[i].vx+boidsNew[i].vx)/2;
				double vy = (boidsOld[i].vy+boidsNew[i].vy)/2;
				double V = sqrt(vx*vx+vy*vy);
				double angleV = angle(0,0,vx,vy);
				double angle12 = angle(x1,y1,x2,y2);
				
				double angleVMirror = 2*angle12-angleV;
				boidsNew[i].vx = V*cos(angleVMirror*PI/180);
				boidsNew[i].vy = V*sin(angleVMirror*PI/180);
				
				// mirror positions
				
				double angle34Mirror = 2*angle12-angle34;
				double dInt4 = d34-d3Int;
				boidsNew[i].x = xInt + dInt4*cos(angle34Mirror*PI/180);
				boidsNew[i].y = yInt + dInt4*sin(angle34Mirror*PI/180);
			}
		}
	}
}

/////////////////////// Wall initialisation functions //////////////////////

void addRandomWall(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                   default_random_engine &gen)
{
	double x1 = boxSizeX * dist01(gen);
	double y1 = boxSizeY * dist01(gen);
	double x2 = boxSizeX * dist01(gen);
	double y2 = boxSizeY * dist01(gen);
	
	Wall wall(x1,y1,x2,y2);
	walls.push_back(wall);
}

void addRandomWallOnSquareGrid(vector<Wall> &walls, int boxSizeX, int boxSizeY,
                               default_random_engine &gen)
{
	// subdivide the box in a Nx by Ny grid of smaller boxes
	// choose Nx,Ny such as the small boxes have a side length of about 5
	// we then place a wall randomly on the edges of this grid
	
	int Nx = int (boxSizeX/5.0);
	int Ny = int (boxSizeY/5.0);
	
	uniform_int_distribution<int> distX(0,Nx);
	uniform_int_distribution<int> distY(0,Ny);
	
	double dx = boxSizeX*1.0/Nx;
	double dy = boxSizeY*1.0/Ny;
	
	double x1 = dx * distX(gen);
	double y1 = dy * distY(gen);
	double x2 = x1;
	double y2 = y1;
	
	double r = dist01(gen);
	
	if (r<0.5)
	{
		while (x2==x1) x2 = dx * distX(gen);
	}
	else
	{
		while (y2==y1) y2 = dy * distY(gen);
	}
	
	Wall wall(x1,y1,x2,y2);
	walls.push_back(wall);
}

////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	/////////////////////////////// Version ////////////////////////////////
	
	string versionCode = "1.2.1";
	
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
			capForce(boids);
			
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
			//renderForces(window,boids,scaleX,scaleY);
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