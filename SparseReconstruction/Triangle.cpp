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

		//Projection Formula
		P_prime_ = P_ - N*((P_ - Z_).dot(N));

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

		is_calculated_ = true;
		BarycentricValidity();
	}

	void Triangle::BarycentricValidity()
	{
		is_barycentric_valid_ = !(BaryCoord_[0] < 0 || BaryCoord_[1] < 0 || BaryCoord_[2] < 0);
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

	double Triangle::NormalRegNorm(Vec3d PNormal)
	{
		if (is_calculated_)
			return PNormal.dot(X_ - Y_)*PNormal.dot(X_ - Y_) +
			PNormal.dot(Y_ - Z_)*PNormal.dot(Y_ - Z_) +
			PNormal.dot(Z_ - X_)*PNormal.dot(Z_ - X_);
		else
			if (is_p_set_)
			{
				CalcResult();
				if (!is_barycentric_valid_)
					std::cout << "Projection not lies in triangle! Use this at your risk!" << std::endl;
				return PNormal.dot(X_ - Y_)*PNormal.dot(X_ - Y_) +
					PNormal.dot(Y_ - Z_)*PNormal.dot(Y_ - Z_) +
					PNormal.dot(Z_ - X_)*PNormal.dot(Z_ - X_);
			}
			else
			{
				std::cout << "No projecting point is set, will return 0!" << std::endl;
				return 0;
			}
	}

	bool Triangle::IsBarycentricValid()
	{
		return is_barycentric_valid_;
	}

	double Triangle::RegEnergy()
	{
		return pow(ProjectedErrorNorm(), 0.5) +
			2.5*EdgeRegSquaredNorm() / 3.0;
	}



}
