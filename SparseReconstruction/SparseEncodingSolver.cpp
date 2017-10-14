#include "SparseEncodingSolver.h"



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
	return 0.0;
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
	return false;
}

bool SparseEncodingSolver::GetSparseEncodingResult(std::vector<TriProj::Triangle>& B)
{
	return false;
}
