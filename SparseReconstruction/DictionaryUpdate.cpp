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
	// This is equivalent to M.norm()
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


/* 
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
	//std::cout << Z_.norm() << std::endl;
	D_.resize(3, wid_P_);
	D_.setZero();
}
*/

DictionaryUpdate::DictionaryUpdate(TriMesh _mesh_)
{
	mesh_ = _mesh_;
	gamma_ = 0.3;
	SetMatrixFromMesh();
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
	for (int i = 0; i < 10; i++)
	{
		PrimalUpdate();
		DualUpdate();
		PenaltyUpdate();
	}
	return mesh_;
}


void DictionaryUpdate::SetMatrixFromMesh()
{
	//Get V,B,P,Z
	wid_V_ = 0;
	wid_P_ = 0;
	for (TriMesh::VertexIter v_it = mesh_.vertices_begin();
		v_it != mesh_.vertices_end(); v_it++)
	{
		if (mesh_.data(*v_it).is_mesh_point_)
		{
			wid_V_++;
			wid_P_++;
		}
	}
	
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
	double z_ = 1.;
	double beta_a = pow((2. * lambda_ * (1. - q)), (1. / (2. - q)));
	double h_a = beta_a + lambda_ * q * pow(beta_a, q - 1); 
	std::cout << "lambda:" << lambda_ << std::endl;
	std::cout << "beta_a:" << beta_a << std::endl;
	std::cout << "h_a:" << h_a << std::endl;
	MatrixXd X_;
	X_= P_ - V_ * B_ - D_ / gamma_;
	std::cout << "norm_Z:" << Z_.norm() << std::endl;
	double beta_;
	for (int i = 0; i < wid_P_; i++)
	{
		double x_norm_ = (X_.col(i)).norm();
		//x close to zero, no need to solve
		if (x_norm_ < 1e-5)
		{
			Z_.col(i) = Vector3d(0, 0, 0);
			continue;
		}
		double lambda_ = pow(x_norm_,q-2) / (wid_P_ * gamma_);
		double beta_a = pow((2. * lambda_ * (1. - q)), (1. / (2. - q)));
		double h_a = beta_a + lambda_ * q * pow(beta_a, q - 1);
		//std::cout << X_.col(i) << std::endl;
		if (z_ <= h_a)
			Z_.col(i) = Vector3d(0, 0, 0);
		else
		{
			beta_ = (z_ + beta_a) / 2;
			//std::cout << "Do some iteration:\n";
			//std::cout << beta_ << std::endl;
			for (int k = 0; k < 5; k++)
			{
				beta_ = z_ - lambda_ * q * pow(beta_, q - 1);
				
			}
			//std::cout << beta_ << std::endl;
			//std::cout << "End some iteration!\n\n";
			Z_.col(i) = beta_ / z_ * X_.col(i);
		}
	}
	std::cout << "norm_Z:" << Z_.norm() << std::endl;

}

void DictionaryUpdate::SolveSubV()
{
	SparseMatrix<double> L(wid_V_, wid_V_); // Laplace matrix of the mesh
	L.setZero();

	int l = mesh_.n_edges();		// number of edges of the mesh

	for (auto it = mesh_.vertices_begin(); it != mesh_.vertices_end(); it++)
	{

		L.insert(it->idx(), it->idx()) = mesh_.valence(*it);
		for (auto vit = mesh_.vv_begin(*it); vit != mesh_.vv_end(*it); vit++)
		{
			//std::cout << it->idx() << ' ' << vit->idx() << std::endl;
			L.coeffRef(it->idx(), vit->idx()) = -1;
			L.coeffRef(vit->idx(), it->idx()) = -1;
		}
	}

	MatrixXd b_;
	b_ = - gamma_ * (Z_ - P_ + D_ / gamma_)* B_.transpose();
	b_.transposeInPlace();
	VectorXd V_x, V_y, V_z, b_x, b_y, b_z;
	b_x = b_.col(0);
	b_y = b_.col(1);
	b_z = b_.col(2);

	SparseMatrix<double> coef_mat;
	coef_mat = omega_e / l * L + gamma_ * B_ * B_.transpose();
	std::cout << "norm_B: " << B_.norm() << std::endl;
	std::cout << "norm_L: " << L.norm() << std::endl;
	std::cout << "norm_b: " << b_.norm() << std::endl;
	std::cout << "norm_coef: " << coef_mat.norm() << std::endl;

	coef_mat.makeCompressed();
	
	SimplicialLLT<SparseMatrix<double>> decomposition(coef_mat);
	V_x = decomposition.solve(b_x).transpose();
	V_y = decomposition.solve(b_y).transpose();
	V_z = decomposition.solve(b_z).transpose();

	std::cout << V_.col(0) << std::endl;
	V_.row(0) = V_x;
	V_.row(1) = V_y;
	V_.row(2) = V_z;
	std::cout << V_.col(0) << std::endl;

	for (auto it = mesh_.vertices_begin(); it != mesh_.vertices_end(); it++)
	{
		//std::cout << V_x[it->idx()] << ' ' << V_y[it->idx()]
		//	<< ' ' << V_z[it->idx()] << std::endl;
		mesh_.set_point(*it, TriMesh::Point(V_x[it->idx()], V_y[it->idx()], V_z[it->idx()]));
	}
	mesh_.update_normals();
	
}

void DictionaryUpdate::PrimalUpdate()
{
	for (int i = 0; i < 1; i++)								
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