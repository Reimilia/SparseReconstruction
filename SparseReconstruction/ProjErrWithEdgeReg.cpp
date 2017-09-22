#include "ProjErrWithEdgeReg.h"

namespace EnergyFunc {

	ProjErrWithEdgeReg::ProjErrWithEdgeReg()
	{
		q_ = 1.0;
		edge_term_weight_ = 1.6;
	}

	ProjErrWithEdgeReg::ProjErrWithEdgeReg(double weight, double qpow)
	{
		edge_term_weight_ = weight;
		q_ = qpow;
	}


	void ProjErrWithEdgeReg::SetEdgeTermWeight(double weight)
	{
		edge_term_weight_ = weight;
	}

	void ProjErrWithEdgeReg::SetQNormWeight(double qpow)
	{
		q_ = qpow;
	}

	double ProjErrWithEdgeReg::CalcEnergy(TriProj::Triangle triangle)
	{
		return pow(triangle.ProjectedErrorNorm(), q_)
			+ edge_term_weight_ *triangle.EdgeRegSquaredNorm();
	}


	ProjErrWithEdgeReg::~ProjErrWithEdgeReg()
	{
	}

}
