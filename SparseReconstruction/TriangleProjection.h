#pragma once

/*
Solve out the following problem：

For a fixed point and triangle

calculate its projected barycentric coordinates :

argmin ||p - (ax+by+cz)||

*/

#include <iostream>
#include "Triangle.h"

namespace  TriProj {
	
	Eigen::Vector3d ProjectionPoint(Eigen::Vector3d P, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z)
	{
		// Calculate the projection point on a plane

		//First get the normal vector
		Eigen::Vector3d N = (Z - Y).cross(X - Y);
		N = N / N.norm();

		//Projection Formula
		return P - N*((P - Z).dot(N));
	}

	Eigen::Vector3d BaryCentricCoord(Eigen::Vector3d P, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z)
	{
		/*
		Supposing we have an arbitrary point P,how can we solve
		the projection problem?

		*/

		//First to run the projection program;

		Eigen::Vector4d P_prime;
		P_prime(0) = 1.0;
		P_prime.tail(3) = ProjectionPoint(P, X, Y, Z);
		Eigen::Matrix3d A;
		A.col(0) = X;
		A.col(1) = Y;
		A.col(2) = Z;

		Eigen::MatrixXd B(4, 3);
		B.row(0) = Eigen::RowVector3d::Ones();
		B.block(1, 0, 3, 3) = A;

		Eigen::Vector3d coef = B.colPivHouseholderQr().solve(P_prime);

		return	coef;
	}
	inline bool IfProjInTriangle(Eigen::Vector3d coef)
	{
		/*
		Just test the coefficient
		*/
		return !(coef[0] < 0 || coef[1] < 0 || coef[2] < 0);
	}

	bool IfProjInTriangle(Eigen::Vector3d P, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z)
	{
		/*
		Test from origion (Although not an efficient way)
		*/
		Eigen::Vector3d Coord_Coef = BaryCentricCoord(P, X, Y, Z);
		return IfProjInTriangle(Coord_Coef);
	}

	Eigen::Vector3d ProjectionErrorVec(Eigen::Vector3d P, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z)
	{
		Eigen::Vector3d bc = BaryCentricCoord(P, X, Y, Z);
		return P - bc[0] * X - bc[1] * Y - bc[2] * Z;
	}

	double ProjectionErrorNorm(Eigen::Vector3d P, Eigen::Vector3d X,
		Eigen::Vector3d Y, Eigen::Vector3d Z)
	{
		return (ProjectionErrorVec(P, X, Y, Z)).norm();
	}

	bool ProjectionTest()
	{
		//Unit test for fun
		typedef Eigen::Vector3d Vec3d;
		Vec3d X(1, 2, 0);
		Vec3d Y(3, 2, 0);
		Vec3d Z(2, 3, 1);
		Vec3d P(2, 3, 0);

		Vec3d Coord_Coef = BaryCentricCoord(P, X, Y, Z);

		std::cout << "Coef: " << Coord_Coef << std::endl;

		std::cout << "This is in/not in the triangle:" << IfProjInTriangle(Coord_Coef) << std::endl;

		system("pause");

		return true;
	}

}



