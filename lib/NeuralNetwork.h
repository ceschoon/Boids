#ifndef NeuralNetwork_H
#define NeuralNetwork_H


#include <iostream>
#include <vector>
#include <random>


using namespace std;



class NeuralNetwork
{
	public:
		
		NeuralNetwork(int Ns=5, int Nl=2, int Np=5, int No=2, int seed=-1)
		: Ns_(Ns), Nl_(Nl), Np_(Np), No_(No)
		{
			// construct default weights and biases
			
			int Nweights = Ns_*Np_ + Np_*Np_*(Nl_-1) + Np_*No_;
			int Nbiases = Np_*Nl_ + No_;
			
			w_ = vector<double>(Nweights,0);
			b_ = vector<double>(Nbiases,0);
			
			// random number generation
			
			if (seed<0)
			{
				random_device true_gen;
				seed = true_gen();
			}
			
			seed_ = seed;
			gen_ = default_random_engine(seed);
		}
		
		~NeuralNetwork() {;}
		
		void initRandom();
		
		void perturbWeights(double mean, double std);
		void perturbBiases(double mean, double std);
		
		vector<double> eval(vector<double> sensors);
		
		int getSeed() {return seed_;}
		
		int getNumSensors() {return Ns_;}
		int getNumLayers() {return Nl_;}
		int getNumNeuronsPerLayer() {return Np_;}
		int getNumOutputs() {return No_;}
		
		double getWeight(int il, int i1, int i2); // i1 index on layer closest to the output
		double getBias(int il, int i);
		
		void setWeight(int il, int i1, int i2, double ww);
		void setBias(int il, int i, double bb);
		
		vector<double> getAllWeights() {return w_;}
		vector<double> getAllBiases() {return b_;}
		
		int saveToFile(string filename);
		int loadFromFile(string filename);
		
	protected:
		
		int Ns_; // number of sensors
		int Nl_; // number of neuron layers
		int Np_; // number of neurons per layer
		int No_; // number of outputs
		
		vector<double> w_; // weights
		vector<double> b_; // biases
		
		// random number generation
		int seed_;
		default_random_engine gen_;
		
		// define type of sigmoid function used
		double sigmoid(double x) {return tanh(x);}
};


#endif //NeuralNetwork_H

