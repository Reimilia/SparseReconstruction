#pragma once
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

class DictionaryUpdate;

typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh3D;

class DictionaryUpdate
{
public:
	DictionaryUpdate( Mesh3D &_mesh_, Eigen::MatrixXd &_V_, const Eigen::SparseMatrix<double> _B_, const Eigen::MatrixXd _P_ );
	~DictionaryUpdate();
	bool									solver();	// the final result
	bool									test();		
	bool									init();


private:
	Mesh3D	 								mesh_;		
	double									gamma_;		// penalty weight 
	double									energy_;	// total energy
	int										wid_P_;		// width of P
	int										wid_V_;		// width of V
	Eigen::MatrixXd							V_;		// dictionary to be optimized
	Eigen::SparseMatrix<double>				B_;		// sparse coding matrix
	Eigen::MatrixXd							P_;		// input signal
	Eigen::MatrixXd							Z_;		// error
	Eigen::MatrixXd							D_;		// Lagrangian function

	double									ComputeEnergy();
	void									SolveSubV();
	void									SolveSubZ();
	void									PrimalUpdate();
	void									DualUpdate();
	void									PenaltyUpdate();
};

