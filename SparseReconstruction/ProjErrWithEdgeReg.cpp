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

double ProjErrWithEdgeReg::CalcEnergy(Eigen::Vector3d Proj_err, Eigen::Vector3d X, Eigen::Vector3d Y, Eigen::Vector3d Z)
{
	return 0.0;
}


ProjErrWithEdgeReg::~ProjErrWithEdgeReg()
{
}
