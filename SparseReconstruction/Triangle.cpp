#include "Triangle.h"
#include <iostream>

using namespace TriProj;

void TriProj::Triangle::CalcResult()
{	// Calculate the projection point on a plane
	if (is_calculated_)
		return;

	//First get the normal vector
	Vec3d N = (Z_ - Y_).cross(X_ - Y_);
	N = N / N.norm();

	//Projection Formula
	P_prime_= P_ - N*((P_- Z_).dot(N));

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

	BaryCoord_ = B.colPivHouseholderQr().solve(P_prime_);

	is_calculated_ = true;
	BarycentricValidity();
}

void TriProj::Triangle::BarycentricValidity()
{
	is_barycentric_valid_ = !(BaryCoord_[0] < 0 || BaryCoord_[1] < 0 || BaryCoord_[2] < 0);
}

TriProj::Triangle::Triangle(Vec3d X, Vec3d Y, Vec3d Z)
{
	is_p_set_ = false;
	is_calculated_ = false;
	X_ = X;
	Y_ = Y;
	Z_ = Z;
}

TriProj::Triangle::Triangle(Vec3d P, Vec3d X, Vec3d Y, Vec3d Z)
{
	is_p_set_ = true;
	is_calculated_ = false;
	X_ = X;
	Y_ = Y;
	Z_ = Z;
	P_ = P;
}


TriProj::Triangle::~Triangle()
{
}

bool TriProj::Triangle::SetTriangle(Vec3d X, Vec3d Y, Vec3d Z)
{
	X_ = X;
	Y_ = Y;
	Z_ = Z;
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

bool TriProj::Triangle::SetProjPoint(Vec3d P)
{
	return false;
}

TriProj::Vec3d TriProj::Triangle::ProjectedPoint()
{
	if(is_calculated_)
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

TriProj::Vec3d TriProj::Triangle::BarycentricCoord()
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

TriProj::Vec3d TriProj::Triangle::ProjectedError()
{
	if (is_calculated_)
		return P_-P_prime_;
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

double TriProj::Triangle::ProjectedErrorNorm()
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

double TriProj::Triangle::EdgeNorm()
{
	return (X_ - Y_).norm() + (Y_ - Z_).norm() + (Z_ - X_).norm();
}

double TriProj::Triangle::NormalNorm(Vec3d PNormal)
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

bool TriProj::Triangle::IsBarycentricValid()
{
	return is_barycentric_valid_;
}
