#pragma once
#include "TriMesh.h"
#include "Eigen\Core"
#include "Eigen\Sparse"

// Generate the mesh we need
/*
	This follows the initialization process and shall determine three things:

	1.	Once get point input, it will first do a resampling process to set up dictionary
	2.	Then it will recursively set up mesh
	3.	Finally it will calculate V and B
*/
class OptMeshInit
{
protected:
	MyMesh		dict_mesh_;
	int			mesh_size_;
	Mat3Xd		dictionary_point_V_;
    Mat3Xd		input_points_P_;
	Mat3Xd		input_normals_N_;
	Eigen::SparseMatrix<double> encoding_B_;
	
	

public:
	OptMeshInit();
	~OptMeshInit();
	
	

};

