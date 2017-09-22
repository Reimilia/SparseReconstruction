#pragma once
#include "TriMesh.h"
#include "TriSet.h"
#include "OptSolverParaSet.h"
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
	int			input_size_;
	
	// kNN parameter
	int			triset_control_number_;

	// dictionary points
	std::vector<Eigen::Vector3d>	mesh_points_;

	// approximated points
	std::vector<Eigen::Vector3d>	query_points_;

	std::function<double(TriProj::Triangle)> energy_func_;

	bool energy_cmp_func_(TriProj::Triangle A, TriProj::Triangle B)
	{
		if (energy_func_(A) < energy_func_(B))
			return true;
		else
			return false;
	}

	void CallDownSampling();

	void FindProjectionRelation();

	bool ManifoldCheck();


private:
	// Pick initial dict for that problem
	bool GenerateInitialDict();
	// Generate sparse encoding
	bool GetInitSparseEncoding(std::vector<TriProj::Triangle> &encoding);
	

public:
	OptMeshInit();
	~OptMeshInit();

	/*Only pass triangles, since sparse matrix will only need for V-subproblem
	We don't necessarily need it in B-subproblem since we will instead maintain
	the prioirty queue, which exactly maintains the mesh topology. And after
	that , the B mesh can be rebuild from the pair (point,face).
	*/
	bool BuildInitialSolution(
		OptSolverParaSet para,
		TriMesh	&initial_mesh,
		std::vector<TriProj::Triangle> &sparse_encoding
		);



};

