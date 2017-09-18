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
	// resampling size
	int			mesh_size_;
	
	// kNN parameter
	int			triset_control_number_;

	
	void CallDownSampling();
	 


public:
	OptMeshInit();
	~OptMeshInit();
	
	
	

};

