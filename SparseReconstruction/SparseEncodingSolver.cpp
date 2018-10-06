#include "SparseEncodingSolver.h"

const double INFINITY_ = 1e10;
SparseEncodingSolver::SparseEncodingSolver()
{
}

SparseEncodingSolver::SparseEncodingSolver(TriMesh mesh)
{
	mesh_ = mesh;
	Init();
}


SparseEncodingSolver::~SparseEncodingSolver()
{
}

void SparseEncodingSolver::Init()
{
	//Generate BST by inserting each encoded triangle
	//into the tree, and we use a user-defined pair class
	//to represent the element in priority-queue.
	sorted_tree_.clear();

	//1. Initialize the energy on face
	for (TriMesh::FaceIter f_it_ = mesh_.faces_begin();
		f_it_ != mesh_.faces_end(); f_it_++)
	{
		mesh_.data(*f_it_).is_calculated = false;
		mesh_.data(*f_it_).face_energy = face_tri_energy(f_it_->idx());
	}
	//2. calculate edge energy and build BST
	for (TriMesh::EdgeIter e_it_ = mesh_.edges_begin();
		e_it_ != mesh_.edges_end(); e_it_++)
	{
		double e = edge_tri_energy(e_it_->idx());
		mesh_.data(*e_it_).energy = e;
		sorted_tree_.insert(__Element(e, e_it_->idx()));
	}
	
}

void SparseEncodingSolver::CheckOneEdge()
{
	//Pick the maximum energy one out
	__Element A = sorted_tree_.select(sorted_tree_.size()-1);
	sorted_tree_.remove(A);
	
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
	// extract the face correspondence first
	TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);
	TriMesh::FaceHandle f1 = mesh_.face_handle(heh_);
	TriMesh::FaceHandle f2 = mesh_.opposite_face_handle(heh_);

	std::vector<int> index_f1 = mesh_.data(f1).index();
	std::vector<int> index_f2 = mesh_.data(f2).index();
	double origin_energy = edge_tri_energy(eh_);

	TriMesh::VertexHandle v1h_ = mesh_.from_vertex_handle(heh_);
	TriMesh::VertexHandle v2h_ = mesh_.to_vertex_handle(heh_);
	TriMesh::VertexHandle v3h_ = mesh_.opposite_vh(heh_); 
	TriMesh::VertexHandle v4h_ = mesh_.opposite_he_opposite_vh(heh_);

	TriMesh::Point v1 = mesh_.point(v1h_);
	TriMesh::Point v2 = mesh_.point(v2h_);
	TriMesh::Point v3 = mesh_.point(v3h_);
	TriMesh::Point v4 = mesh_.point(v4h_);

	double energy1 = 0.0, energy2 = 0.0;
	std::vector<int> new_index_f1;
	std::vector<int> new_index_f2;

	for (size_t i = 0; i < index_f1.size(); i++)
	{
		double x, y;

		//assume we already flip the edge
		//calculate the  energy to see if we make progress
		x = face_tri_energy(index_f1[i], v1, v3, v4);
		y= face_tri_energy(index_f1[i], v2, v3, v4);
		if (x < y)
		{
			energy1 += x;
			new_index_f1.push_back(index_f1[i]);
		}
		else
		{
			energy2 += y;
			new_index_f2.push_back(index_f1[i]);
		}
	}

	for (size_t i = 0; i < index_f2.size(); i++)
	{
		double x, y;

		//assume we already flip the edge
		//calculate the  energy to see if we make progress
		x = face_tri_energy(index_f2[i], v1, v3, v4);
		y = face_tri_energy(index_f2[i], v2, v3, v4);
		if (x < y)
		{
			energy1 += x;
			new_index_f1.push_back(index_f2[i]);
		}
		else
		{
			energy2 += y;
			new_index_f2.push_back(index_f2[i]);
		}
	}
	
	if (energy1 + energy2 < origin_energy)
	{
		//We have make progress
		//update face energy
		mesh_.flip(eh_);

		TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);
		TriMesh::FaceHandle f1 = mesh_.face_handle(heh_);
		TriMesh::FaceHandle f2 = mesh_.opposite_face_handle(heh_);
		
		mesh_.data(f1).set_point_index(new_index_f1);
		mesh_.data(f2).set_point_index(new_index_f2);

		mesh_.data(f1).is_calculated = false;
		mesh_.data(f2).is_calculated = false;
		mesh_.data(f1).face_energy = face_tri_energy(f1);
		mesh_.data(f2).face_energy = face_tri_energy(f2);
		

		UpdatePrioirtyQueue(eh_);
	}
	//otherwise do nothing
}

void SparseEncodingSolver::BoundaryEdgeUpdate(TriMesh::EdgeHandle eh_)
{
	TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);
	if (!mesh_.is_boundary(heh_))
		heh_ = mesh_.opposite_halfedge_handle(heh_);

	TriMesh::HalfedgeHandle prev_heh_ = mesh_.prev_halfedge_handle(heh_);
	TriMesh::HalfedgeHandle next_heh_ = mesh_.next_halfedge_handle(heh_);
	assert(mesh_.is_boundary(prev_heh_));
	assert(mesh_.is_boundary(next_heh_));

	TriMesh::VertexHandle v1h_ = mesh_.from_vertex_handle(heh_);
	TriMesh::VertexHandle v2h_ = mesh_.to_vertex_handle(heh_);
	TriMesh::VertexHandle v3h_ = mesh_.opposite_he_opposite_vh(heh_);
	TriMesh::VertexHandle v3_primeh1_ = mesh_.from_vertex_handle(prev_heh_);
	TriMesh::VertexHandle v3_primeh2_ = mesh_.from_vertex_handle(prev_heh_);



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
		TriMesh::Point v3_prime = mesh_.point(v3_primeh1_);

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
		energy1 = INFINITY_;

	if (next_heh_.is_valid())
	{
		energy2 = 0.0;
		TriMesh::Point v3_prime = mesh_.point(v3_primeh2_);
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
		energy2 = INFINITY_;

	if (mesh_.find_halfedge(v2h_, v3_primeh1_) != TriMesh::InvalidHalfedgeHandle)
		progress1 = false;
	if (mesh_.find_halfedge(v3_primeh2_, v1h_) != TriMesh::InvalidHalfedgeHandle)
		progress2 = false;


	if (!progress1 && !progress2)
		return;
	if ((progress1 && !progress2) || (progress1 && progress2 && energy1<energy2))
	{
		//update virtual test for first triangle (previous halfedge)
		
		TriMesh::FaceHandle new_fh_= mesh_.add_face(v1h_, v2h_, v3_primeh1_);
		mesh_.data(fh_).clear_point_index();
		mesh_.data(new_fh_).clear_point_index();
		//copy index
		mesh_.data(fh_).set_point_index(origin_1);
		mesh_.data(new_fh_).set_point_index(new_1);

		//calculate new energy
		mesh_.data(fh_).is_calculated = false;
		mesh_.data(fh_).face_energy = face_tri_energy(fh_);
		mesh_.data(new_fh_).is_calculated = false;
		mesh_.data(new_fh_).face_energy = face_tri_energy(new_fh_);
		//update edges
		UpdatePrioirtyQueue(eh_);
	}
	else
	{
		//update virtual test for second triangle (next halfedge)

		TriMesh::FaceHandle new_fh_ = mesh_.add_face(v1h_, v2h_, v3_primeh2_);
		mesh_.data(fh_).clear_point_index();
		mesh_.data(new_fh_).clear_point_index();
		//copy index
		mesh_.data(fh_).set_point_index(origin_2);
		mesh_.data(new_fh_).set_point_index(new_2);

		//calculate new energy
		mesh_.data(fh_).is_calculated = false;
		mesh_.data(fh_).face_energy = face_tri_energy(fh_);
		mesh_.data(new_fh_).is_calculated = false;
		mesh_.data(new_fh_).face_energy = face_tri_energy(new_fh_);
		//update edges
		UpdatePrioirtyQueue(eh_);
	}
	
}

void SparseEncodingSolver::UpdatePrioirtyQueue(TriMesh::EdgeHandle eh_)
{
	//The edge must not be the boundary edge here
	assert(!mesh_.is_boundary(eh_));
	mesh_.data(eh_).energy = edge_tri_energy(eh_);

	TriMesh::HalfedgeHandle heh_ = mesh_.halfedge_handle(eh_, 0);
	TriMesh::HalfedgeHandle h1_ = mesh_.next_halfedge_handle(heh_);
	TriMesh::HalfedgeHandle h2_ = mesh_.next_halfedge_handle(heh_);
	
	heh_ = mesh_.opposite_halfedge_handle(heh_);
	TriMesh::HalfedgeHandle h3_ = mesh_.next_halfedge_handle(heh_);
	TriMesh::HalfedgeHandle h4_ = mesh_.next_halfedge_handle(heh_);

	TriMesh::EdgeHandle e1_ = mesh_.edge_handle(h1_);
	TriMesh::EdgeHandle e2_ = mesh_.edge_handle(h2_);
	TriMesh::EdgeHandle e3_ = mesh_.edge_handle(h3_);
	TriMesh::EdgeHandle e4_ = mesh_.edge_handle(h4_);

	UpdateEdgeStatus(e1_);
	UpdateEdgeStatus(e2_);
	UpdateEdgeStatus(e3_);
	UpdateEdgeStatus(e4_);
}

void SparseEncodingSolver::UpdateEdgeStatus(TriMesh::EdgeHandle eh_)
{
	//after topology has changed, one needs to update the energy on those 
	//affected edges, thus leads to the change in BST and mesh energy bundle
	//just maintain propagate influence.
	double energy_ = mesh_.data(eh_).energy;
	int index_ = eh_.idx();
	__Element A(energy_, index_);

	//remove old content
	if (sorted_tree_.contains(A))
		sorted_tree_.remove(A);
	
	//update energy and insert new one
	energy_ = edge_tri_energy(eh_);
	mesh_.data(eh_).energy = energy_;
	
	A = __Element(energy_, index_);

	sorted_tree_.insert(A);

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

	if (heh_.is_valid()&&!mesh_.is_boundary(heh_))
	{
		energy_ += face_tri_energy(heh_);
	}
	heh_ = mesh_.halfedge_handle(eh_, 1);
	if (heh_.is_valid()&&!mesh_.is_boundary(heh_))
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
	if (mesh_.data(fh_).is_calculated) 
		return mesh_.data(fh_).face_energy;
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
	mesh_.data(fh_).is_calculated = true;
	mesh_.data(fh_).face_energy = energy_;
	return energy_;
}

double SparseEncodingSolver::face_tri_energy(TriMesh::Point P, TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z)
{
	TriProj::Triangle T(TriProj::Vec3d(P.data()),TriProj::Vec3d(X.data()),
		TriProj::Vec3d(Y.data()), TriProj::Vec3d(Z.data()));
	return T.RegEnergy();
}

double SparseEncodingSolver::face_tri_energy(int index, TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z)
{
	TriMesh::Point P = mesh_.point(mesh_.vertex_handle(index));
	TriProj::Triangle T(TriProj::Vec3d(P.data()), TriProj::Vec3d(X.data()),
		TriProj::Vec3d(Y.data()), TriProj::Vec3d(Z.data()));
	return T.RegEnergy();
}


double SparseEncodingSolver::face_tri_energy(
	std::vector<int> correspondent_points,
	TriMesh::Point X, TriMesh::Point Y, TriMesh::Point Z)
{
	// This is for the assuming case to update
	TriProj::Triangle T(TriProj::Vec3d(X.data()),
		TriProj::Vec3d(Y.data()), TriProj::Vec3d(Z.data()));
	double energy_ = 0;

	for (size_t i = 0; i < correspondent_points.size(); i++)
	{
		energy_ += face_tri_energy(correspondent_points[i], X, Y, Z);
	}

	return energy_;
}



bool SparseEncodingSolver::SetUpTopology(std::vector<TriProj::Triangle> encode_list)
{
	encoded_B_.clear();
	for (size_t i = 0; i < encode_list.size(); i++)
	{		
		encoded_B_.push_back(encode_list[i]);
	}
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
	
	//Don't forget to update normal
	mesh_.request_face_normals();
	mesh_.request_halfedge_normals();
	mesh_.request_vertex_normals();
	mesh_.update_normals();

	return false;
}

bool SparseEncodingSolver::GetSparseEncodingResult()
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

	//Don't forget to update normal
	mesh_.request_face_normals();
	mesh_.request_halfedge_normals();
	mesh_.request_vertex_normals();
	mesh_.update_normals();

	return false;
}
