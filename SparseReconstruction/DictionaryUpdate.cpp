#pragma once

#include "DictionaryUpdate.h"
#include <math.h>

using namespace Eigen;
double p = 2;
double q = 0.3;
double omega_e = 2.5;
double omega_n = 1.6;
int iteration_bound = 100;



double normpq( const MatrixXd M )	// return the Lpq norm of matrix M
{
	double norm = 0;
	double col_norm = 0;
	for ( int i = 0; i < M.cols(); i++ )
	{
		col_norm = 0;
		for ( int j = 0; j < M.rows(); j++ )
		{
			col_norm += pow(M.coeff(j, i), p);
		}
		norm += pow(col_norm, q / p);
	}
	return pow(norm, 1 / q);
}

double normF( const MatrixXd M )
{
	double norm = 0;
	for ( int i = 0; i < M.cols(); i++ )
	{
		for (int j = 0; j < M.rows(); j++)
		{
			norm += pow(M.coeff(j, i), 2);
		}
	}
	return sqrt(norm);
}


DictionaryUpdate::DictionaryUpdate( TriMesh _mesh_, Eigen::MatrixXd &_V_, const Eigen::SparseMatrix<double> _B_, const Eigen::MatrixXd _P_ )
{
	mesh_ = _mesh_;
	V_ = _V_;
	B_ = _B_;
	P_ = _P_;
	gamma_ = 0.3;
	wid_P_ = P_.cols();
	wid_V_ = V_.cols();

	Z_ = P_ - V_ * B_;

	D_.resize(3, wid_P_);
	D_.setZero();
}

DictionaryUpdate::~DictionaryUpdate()
{

}

bool DictionaryUpdate::init()
{
	return true;
}

bool DictionaryUpdate::test()
{
	return true;
}

TriMesh DictionaryUpdate::solver()
{
	for (int i = 0; i < 100; i++)
	{
		PrimalUpdate();
		DualUpdate();
		PenaltyUpdate();
	}
	return mesh_;
}


double DictionaryUpdate::ComputeEnergy()
{
	double energy_appr = 0;
	energy_appr += normpq(P_ - V_ * B_) / wid_P_;
	double energy_edge = 0;
	for (auto it = mesh_.edges_begin(); it != mesh_.edges_end(); it++)
	{
		energy_edge = pow(mesh_.calc_edge_length(*it), 2);
	}
	energy_edge /= mesh_.n_edges();


	return energy_appr + omega_e * energy_edge;
}

void DictionaryUpdate::SolveSubZ()
{
	double lambda_ = 1 / (wid_P_ * gamma_);
	double z_ = 0.;
	double beta_a = pow((2. * lambda_ * (1. - q)), (1. / (2. - q)));
	double h_a = beta_a + lambda_ * q * pow(beta_a, q - 1); 
	MatrixXd X_;
	X_= P_ - V_ * B_ - D_ / gamma_;

	double beta_;
	for (int i = 0; i < wid_V_; i++)
	{
		z_ = (X_.col(i)).norm();
		if (z_ <= h_a)
			Z_.col(i) = Vector3d(0, 0, 0);
		else
		{
			beta_ = (z_ + beta_a) / 2;
			for (int k = 0; k < 100; k++)
				beta_ = z_ - lambda_ * q * pow(beta_, q - 1);
			Z_.col(i) = beta_ / z_ * X_.col(i);
		}
	}
}

void DictionaryUpdate::SolveSubV()
{
	SparseMatrix<double> L; // Laplace matrix of the mesh
	L.resize(wid_V_, wid_V_);
	L.setZero();

	int l = 0;		// number of edges of the mesh

	for (auto it = mesh_.vertices_begin(); it != mesh_.vertices_end(); it++)
	{

		L.insert(it->idx(), it->idx()) = mesh_.valence(*it);
		for (auto vit = mesh_.vv_begin(*it); vit != mesh_.vv_end(*it); vit++)
		{
			std::cout << it->idx() << ' ' << vit->idx() << std::endl;
			L.coeffRef(it->idx(), vit->idx()) = -1;
			L.coeffRef(vit->idx(), it->idx()) = -1;
		}
	}

	MatrixXd b_;
	b_ = gamma_ * (Z_ - P_ + D_ / gamma_).transpose();
	VectorXd V_x, V_y, V_z, b_x, b_y, b_z;
	b_x = b_.col(0);
	b_y = b_.col(1);
	b_z = b_.col(2);

	SparseMatrix<double> coef_mat;
	coef_mat = 2 * omega_e / l * L + gamma_ * B_ * B_.transpose();

	SparseLU<SparseMatrix<double>> decomposition;
	coef_mat.makeCompressed();
	decomposition.compute(coef_mat);
	//This line crashed down
	V_x = decomposition.solve(b_x).transpose();
	V_y = decomposition.solve(b_y).transpose();
	V_z = decomposition.solve(b_z).transpose();

	V_.row(0) = V_x;
	V_.row(1) = V_y;
	V_.row(2) = V_z;

	for (auto it = mesh_.vertices_begin(); it != mesh_.vertices_end(); it++)
	{
		mesh_.set_point(*it, TriMesh::Point(V_x[it->idx()], V_y[it->idx()], V_z[it->idx()]));
	}
}

void DictionaryUpdate::PrimalUpdate()
{
	for (int i = 0; i < 100; i++)								
	{
		SolveSubZ();
		SolveSubV();
	}
}

void DictionaryUpdate::DualUpdate()
{
	D_ = D_ + gamma_ * (Z_ - P_ + V_ * B_);
}

void DictionaryUpdate::PenaltyUpdate()
{

}