#pragma once
#include "TriProjEnergy.h"


/*

Receive input of a triangle, a point 
and you get the energy form (without normal vector term).

Triangle will basically have three points.
Further this can be replaced by the Polygon class

This is implemented via polymorphism 

*/
namespace EnergyFunc
{
	class ProjErrWithEdgeReg :
		public TriProjEnergy
	{
	protected:
		// the trade off coef for the energy (linear coef)
		double	edge_term_weight_;

		// q-power for the projection norm
		double  q_;

	public:
		ProjErrWithEdgeReg();
		ProjErrWithEdgeReg(double weight, double qpow);

		void SetEdgeTermWeight(double weight);
		void SetQNormWeight(double qpow);
		double CalcEnergy(TriProj::Triangle);

		~ProjErrWithEdgeReg();
	};

}
