#pragma once

#include "Eigen\Core"
#include <queue>

#include "TriMesh.h"
#include "TriSet.h"
#include "OptSolverParaSet.h"
#include "PoissonSampling.h"

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

	// dictionary points
	std::vector<Eigen::Vector3d>		mesh_points_;

	// approximated points
	std::vector<Eigen::Vector3d>		query_points_;
	int *is_mesh_dict;
	std::vector<int> sample_index;

	ANNkd_tree  *kdtree_;


private:
	bool IsTriangleInMesh(TriMesh mesh, TriProj::Triangle triangle, TriMesh::FaceHandle &F );
	bool IsFeasible(TriMesh mesh, TriMesh::VertexHandle X, TriMesh::VertexHandle Y, TriMesh::VertexHandle Z);

	bool ManifoldCheck(TriMesh mesh,TriProj::Triangle triangle);
	// Pick initial dict for that problem
	bool GenerateInitialDict(std::vector<Eigen::Vector3d> input_points);
	// Generate sparse encoding
	
	
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
		std::vector<Eigen::Vector3d> input_points,
		TriMesh	&initial_mesh,
		std::vector<TriProj::Triangle> &sparse_encoding
		);

	bool TestPossionDiskSampling(
		OptSolverParaSet para,
		std::vector<Eigen::Vector3d> input_points,
		TriMesh &initial_mesh
		);
};

