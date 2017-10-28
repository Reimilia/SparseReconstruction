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
	Z_ = P_ - V_ * B_;
	std::cout << Z_.col(1) << std::endl;
	std::cout << Z_.norm() << std::endl;
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
	for (int i = 0; i < 1; i++)
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
	wid_P_ = mesh_.n_vertices();
	for (TriMesh::VertexIter v_it = mesh_.vertices_begin();
		v_it != mesh_.vertices_end(); v_it++)
	{
		if (mesh_.data(*v_it).is_mesh_point_)
	                                                                           	{
			mesh_point_index_.push_back(v_it->idx());
			mesh_.data(*v_it).matrix_column_index = wid_V_;
			wid_V_++;
		}
	}
	std::cout << "Here! 1 : "<< wid_V_ << "\n";
	V_.resize(3, wid_V_);
	int cnt = 0;
	for (TriMesh::VertexIter v_it = mesh_.vertices_begin();
		v_it != mesh_.vertices_end(); v_it++)
	{
		if (mesh_.data(*v_it).is_mesh_point_)
		{
			V_.col(cnt++) = Eigen::Vector3d(mesh_.point(*v_it).data());
		}
	}
	std::cout << "Here! 2 : " << V_.norm() << "\n";

	cnt = 0;
	int cnt_for_check = 0;
	for (TriMesh::FaceIter f_it = mesh_.faces_begin();
		f_it != mesh_.faces_end(); f_it++)
	{
		cnt_for_check += mesh_.data(*f_it).index().size();
	}
	wid_P_ = cnt_for_check;
	P_.resize(3, wid_P_);
	std::vector <Eigen::Triplet<double> > B_coef_;

	for (TriMesh::FaceIter f_it = mesh_.faces_begin();
		f_it != mesh_.faces_end(); f_it++)
	{
		//Pick the face point
		TriMesh::FaceVertexIter fv_ = mesh_.fv_iter(*f_it);
		TriMesh::VertexHandle v1h_ = mesh_.vertex_handle(fv_->idx());
		TriMesh::Point v1 = mesh_.point(v1h_);
		fv_++;
		TriMesh::VertexHandle v2h_ = mesh_.vertex_handle(fv_->idx());
		TriMesh::Point v2 = mesh_.point(mesh_.vertex_handle(fv_->idx()));
		fv_++;
		TriMesh::VertexHandle v3h_ = mesh_.vertex_handle(fv_->idx());
		TriMesh::Point v3 = mesh_.point(mesh_.vertex_handle(fv_->idx()));
		
		std::vector<int> index = mesh_.data(*f_it).index();
		cnt_for_check += index.size();

		for (size_t i = 0; i < index.size(); i++)
		{
			TriMesh::Point pt = mesh_.point(mesh_.vertex_handle(index[i]));
			P_.col(cnt) = Eigen::Vector3d(pt.data());
			//std::cout << P_.col(cnt) << std::endl;
			TriProj::Triangle T(TriProj::Vec3d(pt.data()), TriProj::Vec3d(v1.data()),
				TriProj::Vec3d(v2.data()), TriProj::Vec3d(v3.data()));
			TriProj::Vec3d coord_= T.BarycentricCoord();
			//std::cout << coord_ << std::endl;
			B_coef_.push_back(Eigen::Triplet<double>(
				mesh_.data(v1h_).matrix_column_index, cnt, coord_[0]));
			B_coef_.push_back(Eigen::Triplet<double>(
				mesh_.data(v2h_).matrix_column_index, cnt, coord_[1]));
			B_coef_.push_back(Eigen::Triplet<double>(
				mesh_.data(v3h_).matrix_column_index, cnt, coord_[2]));

			cnt++;
		}
	}
	
	
	//assert(cnt_for_check == wid_P_);

	B_.resize(wid_V_, wid_P_);
	B_.reserve(wid_P_ * 3);
	std::cout << "Here! 3 :" << P_.norm() << "\n";
	B_.setZero();
	B_.setFromTriplets(B_coef_.begin(),B_coef_.end());
	std::cout << "Here! 4 :" << B_.norm() << "\n";
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

	for (TriMesh::VertexIter it = mesh_.vertices_begin(); it != mesh_.vertices_end(); it++)
	{
		if (!mesh_.data(*it).is_mesh_point_)
			continue;
		int index = mesh_.data(*it).matrix_column_index;
		L.insert(index, index) = mesh_.valence(*it);
		for (auto vit = mesh_.vv_begin(*it); vit != mesh_.vv_end(*it); vit++)
		{
			int vit_index = mesh_.data(*vit).matrix_column_index;
			L.coeffRef(index, vit_index) = -1;
			L.coeffRef(vit_index, index) = -1;
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
		if (!mesh_.data(*it).is_mesh_point_)
			continue;
		int index = mesh_.data(*it).matrix_column_index;
		//std::cout << V_x[it->idx()] << ' ' << V_y[it->idx()]
		//	<< ' ' << V_z[it->idx()] << std::endl;
		mesh_.set_point(*it, TriMesh::Point(V_x[index], V_y[index], V_z[index]));
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