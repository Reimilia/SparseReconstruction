#pragma once
// Eigen include
#include "Eigen\Core"
#include "Eigen\Sparse"
#include <vector>

// Inner include 
#include "TriMesh.h"
#include "TriSet.h"

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
	int					mesh_size_;
	
	// kNN parameter
	int					triset_control_number_;

	// The generated result for V(indexs) and B
	




	// TODO: figure out what to do for calling downsampling
	void	CallPointSampling();

	 


public:
	OptMeshInit();
	~OptMeshInit();
	
	bool GetPrebuildMesh(
		std::vector<int>	&sampled_index,
		std::vector<Eigen::Triplet<double>> &sparse_encoding,
		TriMesh &topology
	);

	

};

