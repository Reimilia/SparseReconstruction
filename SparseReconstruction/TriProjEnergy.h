#pragma once

#include "Triangle.h"

namespace EnergyFunc {
	class TriProjEnergy
	{

	public:
		TriProjEnergy();

		virtual double CalcEnergy(TriProj::Triangle);
		virtual ~TriProjEnergy();
	};

	double EnergyWithEdgeNorm(TriProj::Triangle tri, double q_norm_, double edge_weight)
	{
		return pow(tri.ProjectedErrorNorm(), q_norm_)
			+ edge_weight *tri.EdgeRegSquaredNorm() /3.0;
	}

}	


