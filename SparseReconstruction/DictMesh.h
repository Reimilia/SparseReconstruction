#pragma once

// Add library for eigen
#include "Eigen\Core"
#include "Eigen\Sparse"

// Inner include
#include "TriMesh.h"
#include "OptSolverParaSet.h"

/*
This Program shall define the mesh we used in this program, and embed the solver in this problem.
*/

//temporary typedef
// TODO Define some terms in a more efficient way


class DictMesh
{
protected:
	// Reconstruction Mesh from Point Clouds
	TriMesh  recon_mesh_;

	// Mesh size
	int		dict_mesh_size_;

	// Parameter Set
	OptSolverParaSet opt_para_;
	void	SetDefaultOptPara();

	// check if the solution mesh has already get
	bool	is_solver_already_run_ = false;
	
protected:
	
	// Matrices we need for convenience representation
	Mat3Xd dictionary_point_V_;
	Mat3Xd input_points_P_;
	Mat3Xd input_normals_N_;
	Eigen::SparseMatrix<double> encoding_B_; 

	// This will be locked after a successful input was copied
	bool	is_point_cloud_initialized_ = false;
	bool	is_normal_initialized_ = false;
	
	

public:
	// Initialization
	DictMesh();
	// Only point-cloud, parameter are set by default
	DictMesh(Mat3Xd input_data);
	// Point-cloud plus normal
	DictMesh(Mat3Xd input_data, Mat3Xd normal);
	// Destructor Func.
	~DictMesh();

	bool SetInputPointClouds(Mat3Xd input_data);
	bool SetInputNormals(Mat3Xd input_normals);

	bool SetOptSolverPara(OptSolverParaSet para);
	inline OptSolverParaSet GetOptSolverPara() { return opt_para_; }

	//Solve with custom parameter
	bool DictReconstruction(OptSolverParaSet para);
	//Solve with default/settled parameter
	bool DictReconstruction();

	inline bool IsSolverAlreadyRun() { return is_solver_already_run_; }
	bool		GetMeshResult(TriMesh &mesh);
	bool		WriteMeshResult(const char* fouts);


protected:
	
	// Main solver algorithm will be implemented here
	void OptSolverAlgorithm();

	// First step of the solver
	void CallInitialize();

	// Iterative two-steps to update sparse coding and dictionary position
	void CallSparseCoding();
	void CallDictionaryUpdate();

	// For convenience, we need to obtain system energy (i.e. the value of optimized problem)
	// Thus we need implement this method to monitor the process
	double ObtainSystemEnergy();
/*
Unit test implementation here
*/
protected:
	bool	is_test_on_= false;

public:
	bool RunUnitTest();
	bool RunInitilizationTest();
	bool RunSparseCodingTest();
	bool RunDictionaryUpdateTest();
};


