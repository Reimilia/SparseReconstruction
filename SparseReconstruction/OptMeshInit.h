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
	std::vector<Eigen::Vector3d>		query_normals_;
	int *is_mesh_dict;
	std::vector<int> sample_index;
	std::queue <int> queue_points_;
	bool *hash_;
	TriProj::TriSet triset_;

	ANNkd_tree  *kdtree_;

	//Dictionary Mesh
	TriMesh mesh_;
	bool has_mesh_created_;

private:
	bool IsTriangleInMesh(TriProj::Triangle triangle, TriMesh::FaceHandle &F );
	TriMesh::FaceHandle IsFeasible(TriMesh::VertexHandle X, TriMesh::VertexHandle Y, TriMesh::VertexHandle Z);

	//This function will change mesh topology as well
	TriMesh::FaceHandle ManifoldCheck(TriProj::Triangle triangle);
	// Pick initial dict for that problem
	bool GenerateInitialDict(std::vector<Eigen::Vector3d> input_points);
	// Generate sparse encoding
	
	
public:
	OptMeshInit();
	~OptMeshInit();

	void GetResultMesh(TriMesh &mesh);

	
	//After Initialization, each make one pair
	bool PairOneQueryPoint();

	/*Only pass mesh, since we can recover matrix from mesh topology
	*/
	bool BuildInitialSolution(
		OptSolverParaSet para,
		TriMesh initial_mesh
		);

	bool TestPossionDiskSampling(
		OptSolverParaSet para,
		std::vector<Eigen::Vector3d> input_points,
		TriMesh &initial_mesh
		);
};

