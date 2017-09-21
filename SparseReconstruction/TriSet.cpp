#include "TriSet.h"

namespace TriProj
{
	TriSet::TriSet(int kNN_size, std::vector<Eigen::Vector3d> input_points)
	{
		int dim = 3; 
		
		ANNpointArray ANN_input_points;
		ANN_input_points = annAllocPts((int)input_points.size(), dim);
		for (int i = 0; i < input_points.size(); ++i)
		{
			for (int j = 0; j < dim; j++)
			{
				ANN_input_points[i][j] = input_points[i][j];
			}
		}

		//build up kdTree
		kdtree_ = new ANNkd_tree(
			ANN_input_points,
			input_points.size(),
			dim
		);
	}

	TriSet::~TriSet()
	{
		delete kdtree_;
	}

	bool TriSet::GenerateTriangleSet(Eigen::Vector3d query_point, std::vector<Eigen::Vector3i>& trianlge_set)
	{
		if (kNN_size_ < 3)
		{
			std::cerr << "This kNN search size is too small(<3)!\n";
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

		//Generate Triangle Sets
		for (int i = 0; i < kNN_size_; i++)
			for (int j = i+1; j < kNN_size_; j++)
				for (int k = j + 1; k < kNN_size_; k++)
				{
					trianlge_set.push_back(
						Eigen::Vector3i(ANN_index[i], ANN_index[j], ANN_index[k])
					);
				}
		
		return true;
	}

	bool TriSet::GenerateSparseEncoding(Eigen::Vector3d query_point, Eigen::Vector3i & triangle_set, Eigen::Triplet<double>& B_encoding)
	{
		return false;
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

