#include "TriProjEnergy.h"



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
