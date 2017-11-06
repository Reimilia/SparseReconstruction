#include "TriSet.h"

namespace TriProj
{

	TriSet::TriSet()
	{
		kdtree_ = NULL;
	}
	TriSet::TriSet(int kNN_size, std::vector<Eigen::Vector3d> input_points)
	{
		is_initialized_ = false;
		SetQuerySize(kNN_size);
		SetInputPoints(input_points);
		SetupKdTree();
	}

	TriSet::~TriSet()
	{
		if (kdtree_)
		{
			delete kdtree_;
			kdtree_ = NULL;
		}
	}

	bool TriSet::GenerateNearestPointSet(Vec3d query_point, std::vector<int>& point_indexes)
	{
		if (kNN_size_ < 3 || point_set_size_ < kNN_size_)
		{
			std::cerr << "This kNN search size is incorrect!\n";
			return false;
		}

		if (!is_initialized_)
		{
			SetupKdTree();
		}

		ANNidxArray ANN_index;
		ANNdistArray ANN_dist;
		ANNpoint	ANN_point;

		int dim = 3;
		double eps = 0;

		ANN_index = new ANNidx[kNN_size_];
		ANN_dist = new ANNdist[kNN_size_];
		ANN_point = annAllocPt(dim);

		for (int i = 0; i < dim; i++)
			ANN_point[i] = query_point[i];

		kdtree_->annkSearch(
			ANN_point,
			kNN_size_,
			ANN_index,
			ANN_dist,
			eps
		);
		
		for (int i = 0; i < kNN_size_; i++)
			point_indexes.push_back(ANN_index[i]);

		if (ANN_index)
			delete ANN_index;
		if (ANN_dist)
			delete ANN_dist;
		if (ANN_point)
			delete ANN_point;
		return true;
	}


	bool TriSet::GenerateTriangleSet(Vec3d query_point, 
		std::vector<TriProj::Triangle> & triangle_set)
	{
		std::vector <int> point_indexes;
		
		if (!GenerateNearestPointSet(query_point, point_indexes))
			return false;

		if (triangle_set.size() > 0)
			triangle_set.clear();

		//Generate Triangle Sets

		for (int i = 0; i < kNN_size_; i++)
			for (int j = i+1; j < kNN_size_; j++)
				for (int k = j + 1; k < kNN_size_; k++)
				{
					triangle_set.push_back(TriProj::Triangle(
						query_point,
						points_[point_indexes[i]], point_indexes[i],
						points_[point_indexes[j]], point_indexes[j],
						points_[point_indexes[k]], point_indexes[k]
					));
				}
		std::sort(triangle_set.begin(), triangle_set.end());
		return true;
	}

	bool TriSet::GenerateTriangleSet(Vec3d query_point, 
		std::vector<TriProj::Triangle>& triangle_set, 
		bool is_include_query_point,
		int query_point_index)
	{
		std::vector <int> point_indexes;
		if(is_include_query_point) 
			kNN_size_--;
		if (!GenerateNearestPointSet(query_point, point_indexes))
		{
			if(is_include_query_point)  kNN_size_++;
			return false;
		}
		if (is_include_query_point)
		{
			point_indexes.push_back(query_point_index);
			for(size_t i=0;i<point_indexes.size();i++)
				std::cout << point_indexes[i] << std::endl;
			kNN_size_++;
		}
		if (triangle_set.size() > 0)
			triangle_set.clear();

		//Generate Triangle Sets

		for (int i = 0; i < kNN_size_; i++)
			for (int j = i + 1; j < kNN_size_; j++)
				for (int k = j + 1; k < kNN_size_; k++)
				{
					triangle_set.push_back(TriProj::Triangle(
						query_point,
						points_[point_indexes[i]], point_indexes[i],
						points_[point_indexes[j]], point_indexes[j],
						points_[point_indexes[k]], point_indexes[k]
					));
				}
		std::sort(triangle_set.begin(), triangle_set.end());
		return true;
	}


	bool TriSet::SetQuerySize(int k)
	{
		if (k < 3)
		{
			std::cerr << "This kNN search size is too small(<3)!\n";
			return false;
		}
		kNN_size_ = k;
		is_initialized_ = false;
		return true;
	}

	bool TriSet::SetInputPoints(std::vector<Vec3d> input_points)
	{
		points_.clear();
		for (int i = 0; i < input_points.size(); i++)
		{
			points_.push_back(input_points[i]);
		}
		point_set_size_ = points_.size();
		is_initialized_ = false;
		return true;
	}


	void TriSet::SetupKdTree()
	{
		//Set up kdTree here
		int dim = 3;
		point_set_size_ = (int)points_.size();

		ANNpointArray ANN_input_points;
		 
		ANN_input_points = annAllocPts(point_set_size_, dim);

		for (int i = 0; i < point_set_size_; ++i)
		{
			
			for (int j = 0; j < dim; j++)
			{
				ANN_input_points[i][j] = points_[i][j];
			}
		}

		//build up kdTree
		if (kdtree_)
			delete kdtree_;
		kdtree_ = new ANNkd_tree(
			ANN_input_points,
			point_set_size_,
			dim
		);

		is_initialized_ = true;
	}
	bool TriSet::energy_cmp(Triangle A, Triangle B)
	{
		return false;
	}
}

