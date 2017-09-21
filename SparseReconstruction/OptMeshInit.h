#pragma once
#include "TriMesh.h"
#include "Eigen\Core"
#include "Eigen\Sparse"

// Generate the mesh we need
/*
	This follows the initialization process and shall determine three things:

	1.	Once get point input, it will first do a resampling process to set up dictionary
	2.	Then it will recursively set up mesh
	3.	Finally it will calculate B
*/
class OptMeshInit
{
protected:
	// resampling size
	int			mesh_size_;
	
	// kNN parameter
	int			triset_control_number_;

	
	void CallDownSampling();
	void FindProjectionRelation();
	bool ManifoldCheck();

	//Only pass triplet, since sparse matrix will only need for V-subproblem
	//We don't necessarily need it in B-subproblem since we will instead maintain 
	//the prioirty queue, which exactly maintains the mesh topology. And after 
	//that , the B mesh can be rebuild from the pair (point,face).
	std::vector<Eigen::Triplet<double>> GetInitSparseEncoding();


public:
	OptMeshInit();
	~OptMeshInit();

	
	
	
	

};

