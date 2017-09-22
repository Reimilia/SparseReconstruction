#include "TriProjEnergy.h"


namespace EnergyFunc {
	TriProjEnergy::TriProjEnergy()
	{
	}

	double TriProjEnergy::CalcEnergy(TriProj::Triangle tri)
	{
		return tri.ProjectedErrorNorm();
	}


	TriProjEnergy::~TriProjEnergy()
	{
	}


}
