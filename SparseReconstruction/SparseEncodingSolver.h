#pragma once
#include <vector>
#include <map>

#include "Triangle.h"
#include "TriMesh.h"
#include "OptSolverParaSet.h"
#include "SizeBalancedTree.h"

class SparseEncodingSolver
{
protected:
	//We will receive triangle list to replace B
	std::vector <TriProj::Triangle> encoded_B_;
	//We will also need to know the query point queue
	std::vector <Eigen::Vector3d> query_point_;
	TriMesh mesh_;
	OptSolverParaSet para_;

	//This is alternative solution for prioirty queue
	//Since the item in the queue need to be adjusted
	int tree_size_;
	SizeBalancedTree <__Element> sorted_tree_;

	//We will return a mesh as a result
	//Also we need some method to construct 
	//matrix from mesh!


protected:
	//Set up priority queue
	void SetUpSBT();

	//Pick one edge from priority queue
	void CheckOneEdge();

	void EdgeFlipUpdate(TriMesh::EdgeHandle eh_);
	
	void BoundaryEdgeUpdate(TriMesh::EdgeHandle eh_);
	
	void UpdateEdgeStatus(TriMesh::EdgeHandle eh_);


	//Overwrite this function if the energy is changed
	double tri_energy(TriProj::Triangle tri)
	{
		return pow(tri.ProjectedErrorNorm(), para_.q_norm_)
			+ para_.edge_reg_weight_ *tri.EdgeRegSquaredNorm() / 3.0;
	}

	double edge_tri_energy(int edge_handle_index);
	double edge_tri_energy(TriMesh::EdgeHandle eh_);

	double face_tri_energy(int face_handle_index);
	double face_tri_energy(TriMesh::HalfedgeHandle heh_);
	double face_tri_energy(TriMesh::FaceHandle fh_);
	double face_tri_energy(int index,
		TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z);
	double face_tri_energy(std::vector<int> correspondent_index,
		TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z);

public:
	SparseEncodingSolver();
	SparseEncodingSolver(TriMesh mesh);
	~SparseEncodingSolver();

	bool SetUpTopology(std::vector <TriProj::Triangle> encode_list);

	bool SetUpQueryPoints(std::vector <Eigen::Vector3d> query_point);
	
	bool GetSparseEncodingResult(std::vector <TriProj::Triangle> &B);

};

