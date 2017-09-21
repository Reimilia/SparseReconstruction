#pragma once

/*
	For initialization process, we need a triangle set for each given point.

	This set can run kNN search and do

*/

//TODO : Connect to ANN 
#include <vector>
#include <ANN\ANN.h>
#include <Eigen\Sparse>
#include "Triangle.h"

namespace TriProj
{
	class TriSet
	{
	protected:

		int kNN_size_;

		int point_set_size_;

		bool is_initialized_ = false;

		ANNkd_tree	*kdtree_;


	public:
		TriSet(int kNN_size, std::vector<Vec3d> input_points);
		~TriSet();

		//Run KNN to get point indexes
		bool GenerateNearestPointSet(
			Vec3d	query_point,
			std::vector<int>	&point_indexes
		);

		bool GenerateTriangleSet(
			Vec3d	query_point,
			std::vector<Eigen::Vector3i> &trianlge_set
		);

		bool GenerateSparseEncoding(Vec3d query_point,
			std::vector<Eigen::Vector3i> &triangle_set,
			std::vector<Eigen::Triplet<double>> &B_encoding
		);
		
		bool SetQuerySize(int k);

	};
}