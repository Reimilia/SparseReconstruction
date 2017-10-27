#include "SparseEncodingSolver.h"
#define INFINITY 1e10;


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
	TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);
	if (!mesh_.is_boundary(heh_))
		heh_ = mesh_.opposite_halfedge_handle(heh_);

	TriMesh::HalfedgeHandle prev_heh_ = mesh_.prev_halfedge_handle(heh_);
	TriMesh::HalfedgeHandle next_heh_ = mesh_.next_halfedge_handle(heh_);

	TriMesh::VertexHandle v1h_ = mesh_.from_vertex_handle(heh_);
	TriMesh::VertexHandle v2h_ = mesh_.to_vertex_handle(heh_);
	TriMesh::VertexHandle v3h_ = mesh_.opposite_he_opposite_vh(heh_);

	TriMesh::Point v1 = mesh_.point(v1h_);
	TriMesh::Point v2 = mesh_.point(v2h_);
	TriMesh::Point v3 = mesh_.point(v3h_);

	TriMesh::FaceHandle fh_ = mesh_.opposite_face_handle(heh_);

	double energy1, energy2;
	bool progress1= false, progress2 = false;
	std::vector<int> origin_1,new_1,origin_2,new_2;


	if (prev_heh_.is_valid())
	{
		energy1 = 0.0;
		TriMesh::Point v3_prime = mesh_.point(mesh_.from_vertex_handle(prev_heh_));
		std::vector<int> index = mesh_.data(fh_).index();

		for (size_t i = 0; i < index.size(); i++)
		{
			double x, y;
			x = face_tri_energy(index[i], v1, v2, v3);
			y = face_tri_energy(index[i], v1, v2, v3_prime);
			if (y < x)
			{
				energy1 += y;
				new_1.push_back(index[i]);
				progress1 = true;
			}
			else
			{
				energy1 += x;
				origin_1.push_back(index[i]);
			}	
		}
	}
	else
		energy1 = INFINITY;

	if (next_heh_.is_valid())
	{
		energy2 = 0.0;
		TriMesh::Point v3_prime = mesh_.point(mesh_.to_vertex_handle(next_heh_));
		std::vector<int> index = mesh_.data(fh_).index();
		
		for (size_t i = 0; i < index.size(); i++)
		{
			double x, y;
			x = face_tri_energy(index[i], v1, v2, v3);
			y = face_tri_energy(index[i], v1, v2, v3_prime);
			if (y < x)
			{
				energy2 += y;
				progress2 = true;
				new_2.push_back(index[i]);
			}
			else
			{
				energy2 += x;
				origin_2.push_back(index[i]);
			}
		}
	}
	else
		energy2 = INFINITY;

	if (!progress1 && !progress2)
		return;
	if ((progress1 && !progress2) || (progress1 && progress2 && energy1<energy2))
	{
		//update virtual test for first triangle (previous halfedge)
		TriMesh::VertexHandle v3_primeh_ = mesh_.from_vertex_handle(prev_heh_);
		TriMesh::FaceHandle new_fh_= mesh_.add_face(v1h_, v2h_, v3_primeh_);
		mesh_.data(fh_).clear_point_index();
		mesh_.data(new_fh_).clear_point_index();
		//copy index
		for (size_t i = 0; i < origin_1.size(); i++)
		{
			mesh_.data(fh_).add_point_index(origin_1[i]);
			mesh_.data(new_fh_).add_point_index(new_1[i]);
		}
		//calculate new energy
		mesh_.data(fh_).face_energy = face_tri_energy(fh_);
		mesh_.data(new_fh_).face_energy = face_tri_energy(new_fh_);
	}
	else
	{
		//update virtual test for second triangle (next halfedge)
		TriMesh::VertexHandle v3_primeh_ = mesh_.to_vertex_handle(next_heh_);
		TriMesh::FaceHandle new_fh_ = mesh_.add_face(v1h_, v2h_, v3_primeh_);
		mesh_.data(fh_).clear_point_index();
		mesh_.data(new_fh_).clear_point_index();
		//copy index
		for (size_t i = 0; i < origin_1.size(); i++)
		{
			mesh_.data(fh_).add_point_index(origin_2[i]);
			mesh_.data(new_fh_).add_point_index(new_2[i]);
		}
		//calculate new energy
		mesh_.data(fh_).face_energy = face_tri_energy(fh_);
		mesh_.data(new_fh_).face_energy = face_tri_energy(new_fh_);

	}
	
}

void SparseEncodingSolver::UpdateEdgeStatus(TriMesh::EdgeHandle eh_)
{
	//after topology has changed, one needs to update the energy on those 
	//affected faces, thus leads to the change in BST and mesh energy bundle.

	//This is not the function that update the changed edge directly=
	//just maintain propagate influence.


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
		energy_ += face_tri_energy(heh_);
	}
	heh_ = mesh_.halfedge_handle(eh_, 1);
	if (heh_.is_valid())
	{
		energy_ += face_tri_energy(heh_);
	}
	return energy_;
}

double SparseEncodingSolver::face_tri_energy(int face_handle_index)
{
	TriMesh::FaceHandle fh_ = mesh_.face_handle(face_handle_index);
	return face_tri_energy(fh_);
}

double SparseEncodingSolver::face_tri_energy(TriMesh::HalfedgeHandle heh_)
{
	TriMesh::FaceHandle fh_ = mesh_.face_handle(heh_);
	return face_tri_energy(fh_);
}

double SparseEncodingSolver::face_tri_energy(TriMesh::FaceHandle fh_)
{
	double energy_ = 0;
	std::vector<int> s = mesh_.data(fh_).index();
	TriMesh::FaceVertexIter fv_= mesh_.fv_iter(fh_);
	TriMesh::Point v1 = mesh_.point(mesh_.vertex_handle(fv_->idx()));
	fv_++;
	TriMesh::Point v2 = mesh_.point(mesh_.vertex_handle(fv_->idx()));
	fv_++;
	TriMesh::Point v3 = mesh_.point(mesh_.vertex_handle(fv_->idx()));
	for (size_t i = 0; i < s.size(); i++)
	{
		energy_ += face_tri_energy(s[i], v1, v2, v3);
	}
	return energy_;
}

double SparseEncodingSolver::face_tri_energy(int index, TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z)
{
	TriProj::Triangle T(query_point_[index],TriProj::Vec3d(X.data()),
		TriProj::Vec3d(Y.data()), TriProj::Vec3d(Z.data()));
	return T.RegEnergy();
}

double SparseEncodingSolver::face_tri_energy(std::vector<int> correspondent_index, TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z)
{
	// This is for the assuming case to update
	TriProj::Triangle T(TriProj::Vec3d(X.data()),
		TriProj::Vec3d(Y.data()), TriProj::Vec3d(Z.data()));
	double energy_ = 0;

	for (size_t i = 0; i < correspondent_index.size(); i++)
	{
		T.SetProjPoint(query_point_[correspondent_index[i]]);
		energy_ += T.RegEnergy();
	}

	return energy_;
}

SparseEncodingSolver::SparseEncodingSolver()
{
}

SparseEncodingSolver::SparseEncodingSolver(TriMesh mesh)
{
	mesh_ = mesh;
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
		if (mesh_.data(*f_it_).face_energy == 0)
			mesh_.delete_face(*f_it_, false);
	}

	//This will throw out the deleted element from memory.
	mesh_.garbage_collection();
	return false;
}
