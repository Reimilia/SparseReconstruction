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
#include "TriMesh.h"

namespace TriProj
{
	class TriSet
	{
	protected:

		int kNN_size_;

		int point_set_size_;

		bool is_initialized_ = false;

		ANNkd_tree	*kdtree_;
		TriMesh		*aux_mesh_;
		
		std::vector<Eigen::Vector3d> points_;

		

	public:
		TriSet();
		TriSet(int kNN_size, std::vector<Vec3d> input_points);
		~TriSet();

		//Run KNN to get point indexes
		bool GenerateNearestPointSet(
			Vec3d	query_point,
			std::vector<int>	&point_indexes
		);

		bool GenerateTriangleSet(
			Vec3d	query_point,
			std::vector<TriProj::Triangle> &trianlge_set
		);

		bool GenerateTriangleSet(
			Vec3d	query_point,
			std::vector<TriProj::Triangle> &trianlge_set,
			bool	is_include_query_point,
			int		query_point_index
		);


		bool SetQuerySize(int k);
		bool SetInputPoints(std::vector<Vec3d> input_points);
	protected:
		void SetupKdTree();
	private:
		// I don't know if this works
		bool energy_cmp(Triangle X, Triangle Y);
	};
}