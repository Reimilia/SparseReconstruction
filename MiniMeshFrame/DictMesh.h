#pragma once

// Add library support for OpenMesh
//#include <OpenMesh\Core\Mesh\Traits.hh>
//#include <OpenMesh\Core\Mesh\Attributes.hh>
//#include <OpenMesh\Core\Mesh\PolyMesh_ArrayKernelT.hh>
//#include <OpenMesh\Core\Mesh\PolyMeshT.hh>
#include <OpenMesh\Core\Mesh\TriMesh_ArrayKernelT.hh>

// Add library for eigen
#include "Eigen\Core"
#include "Eigen\Sparse"

// Inner include
#include "OptSolverParaSet.h"

/*
This Program shall define the mesh we used in this program, and embed the solver in this problem.
*/

//temporary typedef
// TODO Define some terms in a more efficient way
typedef  Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3Xd;

struct MyTraits : public OpenMesh::DefaultTraits
{
	// use double valued coordinates
	typedef OpenMesh::Vec3d Point;
	// use vertex normals and vertex colors
	VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);

	// store the previous halfedge
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
	// use face normals
	FaceAttributes( OpenMesh::Attributes::Normal );

	// store a face handle for each vertex
	template <class Base, class Refs> struct VertexT : public Base
	{
		int some_additional_index;
	};
};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;

class DictMesh
{
protected:
	// Reconstruction Mesh from Point Clouds
	MyMesh  recon_mesh_;

	// Mesh size
	int		dict_mesh_size_;

	// Parameter Set
	OptSolverParaSet opt_para_;

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
	~DictMesh();

	bool SetInputPointClouds(Mat3Xd input_data);
	bool SetInputNormals(Mat3Xd input_normals);

	//Solve with custom parameter
	bool DictReconstruction(OptSolverParaSet para);
	//Solve with default parameter
	bool DictReconstruction();

protected:
	
	// Main solver algorithm will be implemented here
	void OptSolverAlgorithm();

	// First step of the solver
	void CallInitialize();

	// Iterative two-steps to update sparse coding and dictionary position
	void CallSparseCoding();
	void CallDictionaryUpdate();
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


