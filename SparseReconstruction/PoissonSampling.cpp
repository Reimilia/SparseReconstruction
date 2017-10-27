
#include <cstdlib>
#include <ctime>

#include "PoissonSampling.h"


PoissonSampling::PoissonSampling()
{
}

PoissonSampling::PoissonSampling(std::vector<double*> points)
{ 
	pool_.clear();
	const int oo = 1e7;

	double maxs[3] = { -oo,-oo,-oo }, mins[3] = { oo,oo,oo };

	for (int i = 0; i < points.size(); i++)
	{
		pool_.push_back(points[i]);
		for (int j = 0; j < 3; j++)
		{
			maxs[j] = (maxs[j] < points[i][j] ? points[i][j] : maxs[j]);
			mins[j] = (mins[j] < points[i][j] ? mins[j] : points[i][j]);
		}
	}
	radius_maximum_ = pow((maxs[0] - mins[0])*(maxs[1] - mins[1])*(maxs[2] - mins[2]), 0.33) / 40.0;
	std::cout << radius_maximum_ << std::endl;
	pool_size_ = pool_.size();
}


PoissonSampling::~PoissonSampling()
{
}

bool PoissonSampling::SetSamplingPool(std::vector<double*> points)
{
	pool_.clear();
	const int oo = 1e7;

	double maxs[3] = { -oo,-oo,-oo }, mins[3] = { oo,oo,oo };

	for (int i = 0; i < points.size(); i++)
	{
		pool_.push_back(points[i]);
		for (int j = 0; j < 3; j++)
		{
			maxs[j] = (maxs[j] < points[i][j] ? points[i][j] : maxs[j]);
			mins[j] = (mins[j] < points[i][j] ? mins[j] : points[i][j]);
		}
	}
	radius_maximum_ = pow((maxs[0] - mins[0])*(maxs[1] - mins[1])*(maxs[2] - mins[2]), 0.33) / 10.0;
	pool_size_ = pool_.size();
	return true;
}

bool PoissonSampling::GenerateSamples(int sample_size, std::vector<int>& sample_index)
{
	ANNkd_tree *kdtree;
	ANNpointArray ANN_input_points;
	
	const int dim = 3;

	ANN_input_points = annAllocPts(pool_size_, dim);
	for (int i = 0; i < pool_size_; i++)
		ANN_input_points[i] = pool_[i];

	kdtree = new ANNkd_tree(ANN_input_points, pool_size_, dim);

	bool *hash = new bool[pool_size_];
	
	int pool_remain_size_ = pool_size_;

	//Random seeds
	srand((int)time(0));
	for (int i = 0; i < pool_size_; i++)
		hash[i] = false;

	while (pool_remain_size_ > 0 && sample_index.size()<sample_size)
	{
		int random_sample_index = rand() % pool_size_;
		
		if (hash[random_sample_index])
		{
			//std::cout << pool_remain_size_ << std::endl;
			continue;
		}

		sample_index.push_back(random_sample_index);
		hash[random_sample_index] = true;
		pool_remain_size_--;

		//Remove Samples
		ANNidxArray ANN_index = new ANNidx[1];
		ANNdistArray ANN_dist = new ANNdist[1];
		ANNpoint	ANN_point = annAllocPt(dim);

		for (int i = 0; i < dim; i++)
			ANN_point[i] = pool_[random_sample_index][i];

		//Do a fake search to get number of points within radius
		int k_num = kdtree->annkFRSearch(
			ANN_point,
			radius_,
			0,
			ANN_index,
			ANN_dist,
			0.0
		);
		if (ANN_index)	delete ANN_index;
		if (ANN_dist)	delete ANN_dist;
		ANN_index = new ANNidx[k_num];
		ANN_dist = new ANNdist[k_num];

		kdtree->annkFRSearch(
			ANN_point,
			radius_,
			k_num,
			ANN_index,
			ANN_dist,
			0.0
		);
		for (int i = 0; i < k_num; i++)
		{
			hash[ANN_index[i]] = true;
			pool_remain_size_--;
		}

		if (ANN_index)	delete ANN_index;
		if (ANN_point)	delete ANN_point;
		if (ANN_dist)	delete ANN_dist;

	}

	bool flag = true;

	// Here we need to figure out how to control radius
	if (sample_index.size() < sample_size)
		flag = false;

	if(hash)
		delete[] hash;
	if (kdtree)
		delete kdtree;
	return flag;
}
