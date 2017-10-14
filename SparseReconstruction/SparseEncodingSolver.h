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
	TriMesh mesh_;
	OptSolverParaSet para_;

	//This is alternative solution for prioirty queue
	//Since the item in the queue need to be adjusted
	int queue_size_;
	SizeBalancedTree <double> sorted_tree_;
	int *rank_;

	std::vector<std::vector<int> > point_triangle_;

	//We will return a mesh as a result
	//Also we need some method to construct 
	//matrix from mesh!


protected:
	//Set up priority queue
	void SetUpSBT();

	//Pick one edge from priority queue
	void CheckOneEdge();

	void EdgeFlipUpdate(int edge_handle_index);
	
	void BoundaryEdgeUpdate(int edge_handle_index);
	
	void UpdateEdgeStatus(int edge_handle_index);


	//Overwrite this function if the energy is changed
	double tri_energy(TriProj::Triangle tri)
	{
		return pow(tri.ProjectedErrorNorm(), para_.q_norm_)
			+ para_.edge_reg_weight_ *tri.EdgeRegSquaredNorm() / 3.0;
	}

	double edge_tri_energy(int edge_handle_index);

public:
	SparseEncodingSolver();
	SparseEncodingSolver(std::vector <TriProj::Triangle> encode_list);
	~SparseEncodingSolver();

	bool SetUpTopology(std::vector <TriProj::Triangle> encode_list);
	
	bool GetSparseEncodingResult(std::vector <TriProj::Triangle> &B);

};

