#pragma once

#include <Eigen\Core>
#include <Eigen\Dense>
/*

	Include but not only include:
	1.	projection
	2.  edge_length
	3.	norm_regularization
	4.	barycentric_coordinatess

*/
namespace TriProj
{
	typedef Eigen::Vector3d		Vec3d;

	// customized projection
	class Triangle
	{
	protected:
		//triangle
		Vec3d	X_, Y_, Z_;
		//additional points
		bool	is_p_set_ = false;
		Vec3d	P_;
		
		//additional calculation storage
		bool	is_calculated_ = false;
		Vec3d	P_prime_;
		bool	is_barycentric_valid_ = false;
		Vec3d	BaryCoord_;

		void	CalcResult();
		void	BarycentricValidity();

	public:
		Triangle(Vec3d X, Vec3d Y, Vec3d Z);
		Triangle(Vec3d P, Vec3d X, Vec3d Y, Vec3d Z);
		~Triangle();

		inline Vec3d X() { return X_; };
		inline Vec3d Y() { return Y_; };
		inline Vec3d Z() { return Z_; };

		bool SetTriangle(Vec3d X, Vec3d Y, Vec3d Z);
		bool SetProjPoint(Vec3d P);

		Vec3d ProjectedPoint();
		Vec3d BarycentricCoord();
		Vec3d ProjectedError();
		double ProjectedErrorNorm();
		double EdgeRegNorm();
		double EdgeRegSquaredNorm();
		double NormalRegNorm(Vec3d PNormal);
		bool IsBarycentricValid();

		void Test();
	};


}

