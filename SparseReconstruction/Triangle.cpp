#include "Triangle.h"
#include <iostream>

namespace TriProj 
{
	void Triangle::CalcResult()
	{	// Calculate the projection point on a plane
		if (is_calculated_)
			return;

		//First get the normal vector
		Vec3d N = (Z_ - Y_).cross(X_ - Y_);
		N = N / N.norm();
		N_ = N;

		//Projection Formula
		P_prime_ = P_ - N*((P_ - Z_).dot(N));
		if ((P_ - Z_).dot(N) < 0)
			is_projection_positive_ = false;
		else
			is_projection_positive_ = true;
		//Now calculate barycentric coordinates via 
		Eigen::Vector4d P_prime_ext;
		P_prime_ext(0) = 1.0;
		P_prime_ext.tail(3) = P_prime_;
		Eigen::Matrix3d A;
		A.col(0) = X_;
		A.col(1) = Y_;
		A.col(2) = Z_;

		Eigen::MatrixXd B(4, 3);
		B.row(0) = Eigen::RowVector3d::Ones();
		B.block(1, 0, 3, 3) = A;

		BaryCoord_ = B.colPivHouseholderQr().solve(P_prime_ext);

		//Do some dirty thing here:
		/*
		if (BaryCoord_[1] < zero && BaryCoord_[2] < zero)
		{
			BaryCoord_ = Vec3d(1, 0, 0);
		}
		else
			if (BaryCoord_[0] < zero && BaryCoord_[2] < zero)
			{
				BaryCoord_ = Vec3d(0, 1, 0);
			}
			else
				if (BaryCoord_[0] < zero && BaryCoord_[1] < zero)
				{
					BaryCoord_ = Vec3d(0, 0, 1);
				}
				else
					if (BaryCoord_[0] < zero)
					{
						double lambda = ((Z_ - X_)[0] * (P_prime_ - X_)[1] - (Z_ - X_)[1] * (P_prime_ - X_)[0]) /
							((Y_-Z_)[1] * (P_prime_ - X_)[0] - (Y_ - Z_)[0] * (P_prime_ - X_)[1]);
						if (lambda < 0)
							std::cout << "Wrong!\n";
						BaryCoord_ = Vec3d(0, lambda, 1.0 - lambda);
					}
		else
		if (BaryCoord_[1] < zero)
		{
			double lambda = ((X_ - Y_)[0] * (P_prime_ - Y_)[1] - (X_ - Y_)[1] * (P_prime_ - Y_)[0]) /
				((Z_ - X_)[1] * (P_prime_ - Y_)[0] - (Z_ - X_)[0] * (P_prime_ - Y_)[1]);
			if (lambda < 0)
				std::cout << "Wrong!\n";
			BaryCoord_ = Vec3d(1.0 - lambda, 0, lambda);
		}
		else
		if (BaryCoord_[2] < zero)
		{
			double lambda = ((Y_ - Z_)[0] * (P_prime_ - Z_)[1] - (Y_ - Z_)[1] * (P_prime_ - Z_)[0]) /
				((X_ - Y_)[1] * (P_prime_ - Z_)[0] - (X_ - Y_)[0] * (P_prime_ - Z_)[1]);
			if (lambda < 0)
				std::cout << "Wrong!\n";
			BaryCoord_ = Vec3d(lambda, 1.0 - lambda, 0);
		}*/
		
		is_calculated_ = true;
		BarycentricValidity();
		energy_ = RegEnergy();
	}

	void Triangle::BarycentricValidity()
	{
		is_barycentric_valid_ = !(BaryCoord_[0] < zero || BaryCoord_[1] < zero || BaryCoord_[2] < zero);
	}

	Triangle::Triangle(Vec3d X, Vec3d Y, Vec3d Z)
	{
		is_calculated_ = false;
		X_ = X;
		Y_ = Y;
		Z_ = Z;
	}

	Triangle::Triangle(Vec3d P, Vec3d X, Vec3d Y, Vec3d Z)
	{
		is_p_set_ = true;
		is_calculated_ = false;
		X_ = X;
		Y_ = Y;
		Z_ = Z;
		P_ = P;
		CalcResult();
	}

	Triangle::Triangle(Vec3d P, Vec3d X, int X_id, Vec3d Y, int Y_id, Vec3d Z, int Z_id)
	{
		is_p_set_ = true;
		is_calculated_ = false;
		X_ = X;
		Y_ = Y;
		Z_ = Z;
		P_ = P;
		index_[0] = X_id;
		index_[1] = Y_id;
		index_[2] = Z_id;
		CalcResult();
	}

	Triangle::Triangle(Vec3d P, Vec3d PN, Vec3d X, int X_id, Vec3d Y, int Y_id, Vec3d Z, int Z_id)
	{
		is_p_set_ = true;
		is_calculated_ = false;
		X_ = X;
		Y_ = Y;
		Z_ = Z;
		P_ = P;
		PN_ = PN;
		index_[0] = X_id;
		index_[1] = Y_id;
		index_[2] = Z_id;
		CalcResult();
	}


	Triangle::~Triangle()
	{
	}

	bool Triangle::SetTriangle(Vec3d X, Vec3d Y, Vec3d Z)
	{
		X_ = X;
		Y_ = Y;
		Z_ = Z;
		is_calculated_ = false;

		if (is_p_set_)
			CalcResult();
		else
		{
			std::cout << "Warning: the value of projecting point is not set so far!"
				"Make sure set them otherwise no barycentric coordinates will solve out!" << std::endl;
			return true;
		}
		return true;
	}

	bool Triangle::SetProjPoint(Vec3d P)
	{
		P_ = P;
		is_p_set_ = true;
		is_calculated_ = false;
		CalcResult();
		return true;
	}

	bool Triangle::SetProjNormal(Vec3d PN)
	{
		// This only affects energy calculation
		PN_ = PN;
		is_pn_set_ = true;
		return true;
	}

	Vec3d Triangle::ProjectedPoint()
	{
		if (is_calculated_)
			return P_prime_;
		else
			if (is_p_set_)
			{
				CalcResult();
				return P_prime_;
			}
			else
			{
				std::cout << "No projecting point is set, will return (0,0,0)!" << std::endl;
				return Vec3d(0, 0, 0);
			}
	}

	Vec3d Triangle::BarycentricCoord()
	{
		if (is_calculated_)
			return BaryCoord_;
		else
			if (is_p_set_)
			{
				CalcResult();
				if (!is_barycentric_valid_)
					std::cout << "Projection not lies in triangle! Use this at your risk!" << std::endl;
				return BaryCoord_;
			}
			else
			{
				std::cout << "No projecting point is set, will return (0,0,0)!" << std::endl;
				return Vec3d(0, 0, 0);
			}
	}

	Vec3d Triangle::ProjectedError()
	{
		if (is_calculated_)
			return P_ - P_prime_;
		else
			if (is_p_set_)
			{
				CalcResult();
				if (!is_barycentric_valid_)
					std::cout << "Projection not lies in triangle! Use this at your risk!" << std::endl;
				return  P_ - P_prime_;
			}
			else
			{
				std::cout << "No projecting point is set, will return (0,0,0)!" << std::endl;
				return Vec3d(0, 0, 0);
			}
	}

	double Triangle::ProjectedErrorNorm()
	{
		if (is_calculated_)
			return (P_ - P_prime_).norm();
		else
			if (is_p_set_)
			{
				CalcResult();
				if (!is_barycentric_valid_)
					std::cout << "Projection not lies in triangle! Use this at your risk!" << std::endl;
				return  (P_ - P_prime_).norm();
			}
			else
			{
				std::cout << "No projecting point is set, will return 0!" << std::endl;
				return 0;
			}
	}

	double Triangle::EdgeRegNorm()
	{
		return (X_ - Y_).norm() + (Y_ - Z_).norm() + (Z_ - X_).norm();
	}

	double Triangle::EdgeRegSquaredNorm()
	{
		return (X_ - Y_).squaredNorm() + (Y_ - Z_).squaredNorm() + (Z_ - X_).squaredNorm();
	}

	double Triangle::NormalRegNorm()
	{
		if (!is_pn_set_)
			return 0.0;
		if (is_calculated_)
			return PN_.dot(X_ - Y_)*PN_.dot(X_ - Y_) +
			PN_.dot(Y_ - Z_)*PN_.dot(Y_ - Z_) +
			PN_.dot(Z_ - X_)*PN_.dot(Z_ - X_);
		else
			if (is_p_set_)
			{
				CalcResult();
				if (!is_barycentric_valid_)
					std::cout << "Projection not lies in triangle! Use this at your risk!" << std::endl;
				return PN_.dot(X_ - Y_)*PN_.dot(X_ - Y_) +
					PN_.dot(Y_ - Z_)*PN_.dot(Y_ - Z_) +
					PN_.dot(Z_ - X_)*PN_.dot(Z_ - X_);
			}
			else
			{
				std::cout << "No projecting point is set, will return 0!" << std::endl;
				return 0;
			}
	}

	bool Triangle::IsBarycentricValid()
	{
		if(is_calculated_)
			return is_barycentric_valid_;
		else
		{
			CalcResult();
			return is_barycentric_valid_;
		}
	}

	bool Triangle::IsProjectionPositive()
	{
		return is_projection_positive_;
	}

	double Triangle::RegEnergy()
	{
		if (!IsBarycentricValid())
			return inf;
		return pow(ProjectedErrorNorm(), 0.3) +
			2.5*EdgeRegSquaredNorm()/3.0 + 2.0*NormalRegNorm()/3.0;
	}

	bool Triangle::operator<(const Triangle &X)
	{
		return energy_ < X.energy_;
	}


	bool IsTriTriIntersect(Triangle A, Triangle B)
	{
		/*First rule out the possibility that exactly one edge conincides*/
		int count = 0;
		if (A.X() == B.X() || A.X() == B.Y() || A.X() == B.Z())
			count += 1;
		if (A.Y() == B.X() || A.Y() == B.Y() || A.Y() == B.Z())
			count += 1;
		if (A.Z() == B.X() || A.Z() == B.Y() || A.Z() == B.Z())
			count += 1;
		if (count >= 2)
			return false;


		// Then do a fine check
		TriIntersect::ATriangle T1, T2;

		T1.Vertex_1[0] = A.X()[0];
		T1.Vertex_1[1] = A.X()[1];
		T1.Vertex_1[2] = A.X()[2];

		T1.Vertex_2[0] = A.Y()[0];
		T1.Vertex_2[1] = A.Y()[1];
		T1.Vertex_2[2] = A.Y()[2];

		T1.Vertex_3[0] = A.Z()[0];
		T1.Vertex_3[1] = A.Z()[1];
		T1.Vertex_3[2] = A.Z()[2];

		T1.Normal_0[0] = A.N()[0];
		T1.Normal_0[1] = A.N()[1];
		T1.Normal_0[2] = A.N()[2];

		T2.Vertex_1[0] = B.X()[0];
		T2.Vertex_1[1] = B.X()[1];
		T2.Vertex_1[2] = B.X()[2];

		T2.Vertex_2[0] = B.Y()[0];
		T2.Vertex_2[1] = B.Y()[1];
		T2.Vertex_2[2] = B.Y()[2];

		T2.Vertex_3[0] = B.Z()[0];
		T2.Vertex_3[1] = B.Z()[1];
		T2.Vertex_3[2] = B.Z()[2];

		T2.Normal_0[0] = B.N()[0];
		T2.Normal_0[1] = B.N()[1];
		T2.Normal_0[2] = B.N()[2];

		TriIntersect::TopologicalStructure tag = TriIntersect::judge_triangle_topologicalStructure(&T1, &T2);
		if (tag == TriIntersect::TopologicalStructure::INTERSECT)
			return true;
		else
			return false;
	}
}
