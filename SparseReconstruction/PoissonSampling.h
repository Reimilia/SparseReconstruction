#pragma once

#include <vector>
#include <ANN/ANN.h>


class PoissonSampling
{
protected:
	//Poisson disk radius
	double  radius_;

	double	radius_maximum_=0.0;

	int		pool_size_;

	std::vector<double *> pool_;


public:
	PoissonSampling();
	PoissonSampling(std::vector<double *> points);
	~PoissonSampling();
	bool SetSamplingPool(std::vector<double *> points);
	bool SetRaduis(double r) 
	{
		if (r <= 0 || r>radius_maximum_)
			return false;
		radius_ = r; 
		return true; 
	}

	//Generate how many samples from pools
	bool GenerateSamples(int sample_size_, 
		std::vector<int> &sample_index_);


};

