#include <cmath>
#include <exception>

#include "NeuralNetwork.h"




void NeuralNetwork::initRandom()
{
	// distribution
	normal_distribution<double> distNorm(0,1);
	
	// init weights
	for (int i=0; i<w_.size(); i++) w_[i] = distNorm(gen_);
	
	// init biases
	for (int i=0; i<b_.size(); i++) b_[i] = distNorm(gen_);
}



void NeuralNetwork::perturbWeights(double mean, double std)
{
	// distribution
	normal_distribution<double> distNorm(0,1);
	
	for (int i=0; i<w_.size(); i++) w_[i] += distNorm(gen_)*std + mean;
}



void NeuralNetwork::perturbBiases(double mean, double std)
{
	// distribution
	normal_distribution<double> distNorm(0,1);
	
	for (int i=0; i<b_.size(); i++) b_[i] += distNorm(gen_)*std + mean;
}



vector<double> NeuralNetwork::eval(vector<double> sensors)
{
	vector<double> neurons(Nl_*Np_,0); // value of neurons in layers
	vector<double> outputs(No_,0);     // value of neurons in output
	
	// eval neurons in layers
	for (int il=0; il<Nl_; il++)
	{
		for (int ip=0; ip<Np_; ip++)
		{
			double sum = 0;
			
			// sum weighted values of previous neurons
			// if first layer, sum over sensors
			if (il==0) for (int is=0; is<Ns_; is++)
			{
				double w = getWeight(il,ip,is);
				sum += w * sensors[is];
			}
			// else sum over layer-1
			else for (int i2=0; i2<Np_; i2++)
			{
				double w = getWeight(il,ip,i2);
				sum += w * neurons[(il-1)*Np_+i2];
			}
			
			// add bias
			sum += getBias(il,ip);
			
			// store computed value
			neurons[il*Np_+ip] = sigmoid(sum);
		}
	}
	
	// eval output neurons
	for (int io=0; io<No_; io++)
	{
		double sum = 0;
		
		// sum weighted values of previous neurons
		for (int i2=0; i2<Np_; i2++)
		{
			double w = getWeight(Nl_,io,i2);
			sum += w * neurons[(Nl_-1)*Np_+i2];
		}
		
		// add bias
		sum += getBias(Nl_,io);
		
		// store computed value
		outputs[io] = sigmoid(sum);
	}
}



double NeuralNetwork::getWeight(int il, int i1, int i2) 
{
	// test for misuse
	if (il<0 || il>Nl_) throw std::out_of_range("Wrong index in getWeight");
	if (il==0)
	{
		if (i1<0 || i1>=Np_) throw std::out_of_range("Wrong index in getWeight");
		if (i2<0 || i2>=Ns_) throw std::out_of_range("Wrong index in getWeight");
	}
	else if (il<Nl_)
	{
		if (i1<0 || i1>=Np_) throw std::out_of_range("Wrong index in getWeight");
		if (i2<0 || i2>=Np_) throw std::out_of_range("Wrong index in getWeight");
	}
	else
	{
		if (i1<0 || i1>=No_) throw std::out_of_range("Wrong index in getWeight");
		if (i2<0 || i2>=Np_) throw std::out_of_range("Wrong index in getWeight");
	}
	
	// return weight
	if (il==0) return w_[Ns_*i1+i2];
	else if (il<Nl_) return w_[Ns_*Np_ + Np_*Np_*(il-1) + Np_*i1+i2];
	else return w_[Ns_*Np_ + Np_*Np_*(Nl_-1) + Np_*i1+i2];
}



double NeuralNetwork::getBias(int il, int i)
{
	// test for misuse
	if (il<0 || il>Nl_) throw std::out_of_range("Wrong index in getBias");
	if (il<Nl_)
	{
		if (i<0 || i>=Np_) throw std::out_of_range("Wrong index in getBias");
	}
	else
	{
		if (i<0 || i>=No_) throw std::out_of_range("Wrong index in getBias");
	}
	
	// return weight
	if (il<Nl_) return b_[Np_ + Np_*(il-1) +i];
	else return b_[Np_ + Np_*(Nl_-1) + i];
}



