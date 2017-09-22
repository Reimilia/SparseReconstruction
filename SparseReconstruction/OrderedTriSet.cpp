
#include <algorithm>

//self- including 
#include "stdafx.h"
#include "OrderedTriSet.h"

namespace TriProj
{
	OrderedTriSet::OrderedTriSet()
	{
	}
	OrderedTriSet::OrderedTriSet(OptSolverParaSet para, std::vector<Vec3d> input_points)
	{
		triangle_set_ = new TriSet(para.triangle_set_control_num_, input_points);

		//copy the data
		point_size_ = input_points.size();

		for (int i = 0; i < point_size_; i++)
		{
			points_.push_back(input_points[i]);
		}

		energy_func_ = para.energy_func_;

		is_initialized_ = true;
	}


	OrderedTriSet::~OrderedTriSet()
	{
		if (triangle_set_)
			delete triangle_set_;
	}

	bool OrderedTriSet::GenerateOrderedTriSets(
		Vec3d query_points, std::vector<TriProj::Triangle>& ordered_tri_set)
	{
		std::vector<Eigen::Vector3i> triangle_indexes;
		triangle_set_->GenerateTriangleSet(query_points, triangle_indexes);

		//Build set with indexes
		if (!ordered_tri_set.empty())
			ordered_tri_set.clear();

		for (int i = 0; i < triangle_indexes.size(); i++)
			ordered_tri_set.push_back(TriProj::Triangle(
				query_points,
				points_[triangle_indexes[i][0]],
				points_[triangle_indexes[i][1]],
				points_[triangle_indexes[i][2]]
				));
		//Sort the set
		sort(ordered_tri_set.begin(), ordered_tri_set.end(), energy_cmp_func_);
		
	}

}

