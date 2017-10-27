#pragma once
#include <iostream>

#include "TriMesh.h"
#include <Eigen/Sparse>
#include <Eigen/Cholesky>

class DictionaryUpdate;


class DictionaryUpdate
{
public:
	//DictionaryUpdate( TriMesh _mesh_, Eigen::MatrixXd &_V_, const Eigen::SparseMatrix<double> _B_, const Eigen::MatrixXd _P_ );
	DictionaryUpdate(TriMesh mesh_);
	~DictionaryUpdate();
	TriMesh									solver();	// the final result
	bool									test();		
	bool									init();


private:
	TriMesh	 								mesh_;		
	double									gamma_;		// penalty weight 
	double									energy_;	// total energy
	int										wid_P_;		// width of P
	int										wid_V_;		// width of V
	Eigen::MatrixXd							V_;		// dictionary to be optimized
	Eigen::SparseMatrix<double>				B_;		// sparse coding matrix
	Eigen::MatrixXd							P_;		// input signal
	Eigen::MatrixXd							Z_;		// error
	Eigen::MatrixXd							D_;		// Lagrangian function

	void									SetMatrixFromMesh();
	double									ComputeEnergy();
	void									SolveSubV();
	void									SolveSubZ();
	void									PrimalUpdate();
	void									DualUpdate();
	void									PenaltyUpdate();
};

