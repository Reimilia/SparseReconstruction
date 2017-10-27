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

		int		index_[3];

		void	CalcResult();
		void	BarycentricValidity();

	public:
		Triangle(Vec3d X, Vec3d Y, Vec3d Z);
		Triangle(Vec3d P, Vec3d X, Vec3d Y, Vec3d Z);
		Triangle(Vec3d P, Vec3d X, int X_id,
			Vec3d Y, int Y_id, Vec3d Z, int Z_id);
		~Triangle();

		void GetTriangle(Vec3d &X, Vec3d &Y, Vec3d &Z)
		{
			X = X_;
			Y = Y_;
			Z = Z_;
		}

		void GetTrianglePointIndex(int &X_id, int &Y_id, int &Z_id)
		{
			X_id = index_[0];
			Y_id = index_[1];
			Z_id = index_[2];
		}
		
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
		double RegEnergy();
		
	};


	class Cmp_Triangle : public std::binary_function
		<Triangle, Triangle,bool> 
	{
	protected:
		double q_norm = 0.5;
		double edge_term_ = 2.5;
		double normal_term_ = 0.0;
		double energy_func_(Triangle t)
		{
			return pow(t.ProjectedErrorNorm(), q_norm) +
				edge_term_*t.EdgeRegSquaredNorm() / 3.0;
		}
	public:
		bool operator()(const Triangle &t1, const Triangle &t2)
		{
			if (energy_func_(t1) < energy_func_(t2))
				return true;
			else
				return false;
		}

	};

}

