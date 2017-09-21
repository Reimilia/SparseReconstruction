#include "ProjErrWithEdgeReg.h"



ProjErrWithEdgeReg::ProjErrWithEdgeReg()
{
}

ProjErrWithEdgeReg::ProjErrWithEdgeReg(double weight)
{
	edge_term_weight_ = weight;
}

void ProjErrWithEdgeReg::SetEdgeTermWeight(double weight)
{
	edge_term_weight_ = weight;
}

double ProjErrWithEdgeReg::CalcEnergy(TriProj::Triangle)
{
	return 0.0;
}


ProjErrWithEdgeReg::~ProjErrWithEdgeReg()
{
}
