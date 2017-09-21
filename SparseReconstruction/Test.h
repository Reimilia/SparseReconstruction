#pragma once
// Add library support for OpenMesh
#include	"TriMesh.h"
#include	"DictionaryUpdate.h"
#include	<Eigen/Sparse>

class TestDicUpdate;

class TestDicUpdate
{
public:
	TestDicUpdate(TriMesh _mesh_);
	~TestDicUpdate();
	TriMesh							solver();  // Return the Mesh after Dictionary Update

private:
	void							compute();	// Compute P, B, V for Dictionary Update 

	TriMesh							mesh_;
	Eigen::MatrixXd					P_;
	Eigen::SparseMatrix<double>		B_;
	Eigen::MatrixXd					V_;


};
