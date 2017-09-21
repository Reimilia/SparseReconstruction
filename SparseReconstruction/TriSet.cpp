#include "TriSet.h"

namespace TriProj
{
	TriSet::TriSet(int kNN_size, std::vector<Eigen::Vector3d> input_points)
	{
		int dim = 3; 
		point_set_size_ = (int)input_points.size();

		ANNpointArray ANN_input_points;
	
		ANN_input_points = annAllocPts(point_set_size_, dim);
		for (int i = 0; i < point_set_size_; ++i)
		{
			for (int j = 0; j < dim; j++)
			{
				ANN_input_points[i][j] = input_points[i][j];
			}
		}

		//build up kdTree
		kdtree_ = new ANNkd_tree(
			ANN_input_points,
			point_set_size_,
			dim
		);
	}

	TriSet::~TriSet()
	{
		delete kdtree_;
	}

	bool TriSet::GenerateNearestPointSet(Vec3d query_point, std::vector<int>& point_indexes)
	{
		if (kNN_size_ < 3 || point_set_size_ < kNN_size_)
		{
			std::cerr << "This kNN search size is incorrect!\n";
			return false;
		}
		ANNidxArray ANN_index;
		ANNdistArray ANN_dist;
		ANNpoint	ANN_point;

		int dim = 3;
		double eps = 1e-4;

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

		return true;
	}


	bool TriSet::GenerateTriangleSet(Vec3d query_point, std::vector<int [3]>& trianlge_set)
	{
		std::vector <int> point_indexes;
		
		if (!GenerateNearestPointSet(query_point, point_indexes))
			return false;

		if (trianlge_set.size() > 0)
			trianlge_set.clear();

		//Generate Triangle Sets
		for (int i = 0; i < kNN_size_; i++)
			for (int j = i+1; j < kNN_size_; j++)
				for (int k = j + 1; k < kNN_size_; k++)
				{
					trianlge_set.push_back(
					{ point_indexes[i], point_indexes[j], point_indexes[k] }
					);
				}
		
		return true;
	}

	bool TriSet::GenerateSparseEncoding(Vec3d query_point, 
		std::vector<int [3]>& triangle_set, 
		std::vector<Eigen::Triplet<double>>& B_encoding)
	{
		if (!GenerateTriangleSet(query_point, triangle_set))
			return false;

		for (int i = 0; i < triangle_set.size(); i++)
		{

		}

		Triangle *tri=  new Triangle(query_point,)
	}



	bool TriSet::SetQuerySize(int k)
	{
		if (k < 3)
		{
			std::cerr << "This kNN search size is too small(<3)!\n";
			return false;
		}
		kNN_size_ = k;
		return true;
	}

}

