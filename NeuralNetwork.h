#ifndef NeuralNetwork
#define NeuralNetwork


#include <iostream>
#include <exception>
#include <vector>


using namespace std;


class NeuralNetwork
{
	public:
		
		NeuralNetwork(int Ns=5, int Nl=2, int Np=5)
		: Ns_(Ns), Nl_(Nl), Np_(Np)
		{
			// construct default weights and biases
			
			int Nweights = Ns_*Np_ + Np_*Np_*(Nl_-1) + Np_*No_;
			int Nbiases = Np_*Nl_ + No_;
			
			w_ = vector<double>(Nweights,0);
			b_ = vector<double>(Nbiases,0);
		}
		
		~NeuralNetwork() {;}
		
		// TODO: implementations
		void initRandom();
		void perturb(); // take avg shift and std deviation ?
		
		vector<double> eval(vector<double> sensors);
		
		int getNumSensors() {return Ns_};
		int getNumLayers() {return Nl_};
		int getNumNeuronsPerLayer() {return Np_};
		
		double getWeight(int il, int i1, int i2);
		double getBias(int il, int i);
		
		vector<double> getAllWeights() {return w_};
		vector<double> getAllBiases() {return b_};
		
	protected:
		
		int Ns_; // number of sensors
		int Nl_; // number of neuron layers
		int Np_; // number of neurons per layer
		int No_; // number of outputs
		
		vector<double> w_; // weights
		vector<double> b_; // biases
};


#endif //NeuralNetwork

