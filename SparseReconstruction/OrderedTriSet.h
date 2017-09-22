#pragma once
#include "TriSet.h"
#include "OptSolverParaSet.h"

namespace TriProj {
	class OrderedTriSet :
		public TriSet
	{
	protected:
		std::vector<Vec3d>	points_;

		TriSet	*triangle_set_=NULL;

		int		point_size_=0;

		bool	is_initialized_ = false;

		std::function<double(TriProj::Triangle)> energy_func_;

		bool energy_cmp_func_(Triangle A, Triangle B)
		{
			if (energy_func_(A) < energy_func_(B))
				return true;
			else
				return false;
		}

	public:
		OrderedTriSet();
		OrderedTriSet(OptSolverParaSet para, std::vector<Vec3d> input_points);
		~OrderedTriSet();

		bool GenerateOrderedTriSets(
			Vec3d		query_points,
			std::vector<TriProj::Triangle> &triangle_set
		);
	};

}

