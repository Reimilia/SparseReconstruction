#include "SparseEncodingSolver.h"



void SparseEncodingSolver::SetUpSBT()
{
	//Generate SBT by inserting each one 
}

void SparseEncodingSolver::CheckOneEdge()
{
	//Pick the maximum energy one out
}

void SparseEncodingSolver::EdgeFlipUpdate(int edge_handle_index)
{
}

void SparseEncodingSolver::BoundaryEdgeUpdate(int edge_handle_index)
{
}

void SparseEncodingSolver::UpdateEdgeStatus(int edge_handle_index)
{
}

double SparseEncodingSolver::edge_tri_energy(int edge_handle_index)
{
	
}

SparseEncodingSolver::SparseEncodingSolver()
{
}

SparseEncodingSolver::SparseEncodingSolver(std::vector<TriProj::Triangle> encode_list)
{
}


SparseEncodingSolver::~SparseEncodingSolver()
{
}

bool SparseEncodingSolver::SetUpTopology(std::vector<TriProj::Triangle> encode_list)
{
	encoded_B_.clear();
	for (size_t i = 0; i < encode_list.size(); i++)
		encoded_B_.push_back(encode_list[i]);
}

bool SparseEncodingSolver::SetUpQueryPoints(std::vector<Eigen::Vector3d> query_point)
{
	query_point_.clear();
	for (size_t i = 0; i < query_point.size(); i++)
		query_point_.push_back(query_point[i]);
}

bool SparseEncodingSolver::GetSparseEncodingResult(std::vector<TriProj::Triangle>& B)
{
	return false;
}
