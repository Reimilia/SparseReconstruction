#pragma once
#include "EnergyFunc.h"


/*

Receive input of a triangle, a point 
and you get the energy form (without normal vector term).

Triangle will basically three points.

This is implemented via polymorphism 

*/

class ProjErrWithEdgeReg :
	public EnergyFunc
{
protected:
	double	edge_term_weight_ = 1.0;
	

public:
	ProjErrWithEdgeReg();
	ProjErrWithEdgeReg(double weight);

	void SetEdgeTermWeight(double weight);
	double CalcEnergy(Eigen::Vector3d Proj_err, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z);

	~ProjErrWithEdgeReg();
};
