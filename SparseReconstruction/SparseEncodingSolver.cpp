#include "SparseEncodingSolver.h"



void SparseEncodingSolver::SetUpSBT()
{
	//Generate BST by inserting each encoded triangle
	//into the tree, and we use a user-defined pair class
	//to represent the element in priority-queue.


}

void SparseEncodingSolver::CheckOneEdge()
{
	//Pick the maximum energy one out
	__Element A = sorted_tree_.select(sorted_tree_.size());
	
	TriMesh::EdgeHandle eh_ = mesh_.edge_handle(A.index());

	if (mesh_.is_boundary(eh_))
	{	
		BoundaryEdgeUpdate(eh_);
	}
	else 
	{
		if (mesh_.is_flip_ok(eh_))
		{
			EdgeFlipUpdate(eh_);
		}
	}
}

void SparseEncodingSolver::EdgeFlipUpdate(TriMesh::EdgeHandle eh_)
{
}

void SparseEncodingSolver::BoundaryEdgeUpdate(TriMesh::EdgeHandle eh_)
{
}

void SparseEncodingSolver::UpdateEdgeStatus(TriMesh::EdgeHandle eh_)
{
}

double SparseEncodingSolver::edge_tri_energy(int edge_handle_index)
{
	TriMesh::EdgeHandle eh_= mesh_.edge_handle(edge_handle_index);
	return edge_tri_energy(eh_);
}

double SparseEncodingSolver::edge_tri_energy(TriMesh::EdgeHandle eh_)
{
	TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);

	double energy_ = 0;

	if (heh_.is_valid())
	{
		TriMesh::FaceHandle fh_ = mesh_.face_handle(heh_);
		std::vector<int> s = mesh_.data(fh_).index();

		for (size_t i = 0; i < s.size(); i++)
		{
			energy_ += tri_energy(encoded_B_[s[i]]);
		}
	}
	heh_ = mesh_.halfedge_handle(eh_, 1);
	if (heh_.is_valid())
	{
		TriMesh::FaceHandle fh_ = mesh_.face_handle(heh_);
		std::vector<int> s = mesh_.data(fh_).index();

		for (size_t i = 0; i < s.size(); i++)
		{
			energy_ += tri_energy(encoded_B_[s[i]]);
		}
	}
	return energy_;
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
	return true;
}

bool SparseEncodingSolver::SetUpQueryPoints(std::vector<Eigen::Vector3d> query_point)
{
	query_point_.clear();
	for (size_t i = 0; i < query_point.size(); i++)
		query_point_.push_back(query_point[i]);
	return true;
}

bool SparseEncodingSolver::GetSparseEncodingResult(std::vector<TriProj::Triangle>& B)
{
	// Lower the energy through edge-based operation
	while (!sorted_tree_.empty())
	{
		CheckOneEdge();
	}

	// See if we can remove triangle with zero energy
	mesh_.request_edge_status();
	mesh_.request_face_status();
	mesh_.request_vertex_status();
	
	for (TriMesh::FaceIter f_it_ = mesh_.faces_begin();
		f_it_ != mesh_.faces_end(); f_it_++)
	{
		if (mesh_.data(f_it_).face_energy == 0)
			mesh_.delete_face(f_it_, false);
	}

	//This will throw out the deleted element from memory.
	mesh_.garbage_collection();
	return false;
}
