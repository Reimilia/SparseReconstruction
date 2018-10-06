#include "OptMeshSubdiv.h"



bool OptMeshSubdiv::intialize_tetrahedron()
{
	/* Add four points into the mesh */
	int prev_size = mesh_.n_vertices();
	TriMesh::VertexHandle v0, v1, v2, v3;

	/*
		Assumption here is that the mesh is normalized so it contains in the cube
		[0,1]^3, otherwise this tetrahedron is not true
	*/
	v0 = mesh_.add_vertex(TriMesh::Point(3.0, -1.0, -1.0));
	v1 = mesh_.add_vertex(TriMesh::Point(-1.0, 3.0, -1.0));
	v2 = mesh_.add_vertex(TriMesh::Point(-1.0, -1.0, 3.0));
	v3 = mesh_.add_vertex(TriMesh::Point(-1.0, -1.0, -1.0));
	
	mesh_.data(v0).is_mesh_point_ = true;
	mesh_.data(v1).is_mesh_point_ = true;
	mesh_.data(v2).is_mesh_point_ = true;
	mesh_.data(v3).is_mesh_point_ = true;
	
	std::vector<int> idxs;
	for (int i = 0; i < prev_size; i++)
	{
		idxs.push_back(i);
	}

	// Assign every point in the face:

	TriMesh::FaceHandle fh;
	fh = mesh_.add_face(v3, v0, v2);
	reset_assignment(fh, idxs);

	fh = mesh_.add_face(v3, v1, v0);
	reset_assignment(fh, idxs);

	fh = mesh_.add_face(v3, v2, v1);
	reset_assignment(fh, idxs);
	
	fh = mesh_.add_face(v0, v1, v2);
	reset_assignment(fh, idxs);


	return true;
}

bool OptMeshSubdiv::reset_assignment(TriMesh::FaceHandle fh, std::vector<int> idxs)
{
	// Key idea is to calculate barycentric coordinate for each point
	// to see if it is valid.
	// also calculate face energy and store in each face.
	TriMesh::FaceVertexIter fv_iter = mesh_.fv_iter(fh);
	TriMesh::Point x, y, z;
	x = mesh_.point(*fv_iter);
	if ((++fv_iter).is_valid()) y = mesh_.point(*fv_iter);
	if ((++fv_iter).is_valid()) z = mesh_.point(*fv_iter);
	TriProj::Triangle T(
		TriProj::Vec3d(x.data()),
		TriProj::Vec3d(y.data()),
		TriProj::Vec3d(z.data())
	);
	mesh_.data(fh).clear_point_index();
	double energy = 0.0;
	for (std::size_t i = 0; i < idxs.size(); i++)
	{
		TriMesh::VertexHandle v = mesh_.vertex_handle(idxs[i]);
		T.SetProjPoint(TriProj::Vec3d(mesh_.point(v).data()));
		if (T.IsBarycentricValid())
		{
			energy += T.RegEnergy();
			mesh_.data(fh).add_point_index(idxs[i]);
		}
	}
	mesh_.data(fh).face_energy = energy;
	return true;
}


bool OptMeshSubdiv::subdivide()
{
	std::vector <bool> status;
	for (TriMesh::FaceIter f_it = mesh_.faces_begin();
		f_it != mesh_.faces_end(); ++f_it)
	{
		TriMesh::FaceHandle fh = mesh_.face_handle(f_it->idx());
		double proj_error = face_tri_energy(fh);
		if (proj_error > threshold_)
		{
			std::cout << "Energy error: " << proj_error << std::endl;
			status.push_back(true);
		}
		else
			status.push_back(false);
	}
	size_t i = 0;
	for (TriMesh::FaceIter f_it = mesh_.faces_begin();
		i < status.size(); ++f_it, ++i)
	{
		TriMesh::FaceHandle fh = mesh_.face_handle(f_it->idx());
		if (status[i])
		{
			assign_points(fh);
		}
	}
	mesh_.garbage_collection();
	return true;
}

bool OptMeshSubdiv::topologyupdate()
{
	SparseEncodingSolver *solver = new SparseEncodingSolver(mesh_);

	solver->GetSparseEncodingResult();
	mesh_ = solver->GetMesh();

	delete solver;

	return false;
}


bool OptMeshSubdiv::vertexupdate()
{
	// Call DictionaryUpdage class to solve optimal V, (for fixed B)
	DictionaryUpdate  *solver = new DictionaryUpdate(mesh_);
	
	mesh_ = solver->solver();

	delete solver;

	// Update each face by reassigning all points (which is necessary since
	// energy has changed)

	return false;
}

bool OptMeshSubdiv::assign_points(TriMesh::FaceHandle fh)
{
	//1. split into three
	//2. remove original face
	//3. add three new face and assign accordingly.


	TriMesh::FaceVertexIter fv_iter = mesh_.fv_iter(fh);
	TriMesh::VertexHandle w, x, y, z;
	x = mesh_.vertex_handle(fv_iter->idx());
	if ((++fv_iter).is_valid()) {
		y = mesh_.vertex_handle(fv_iter->idx());
	}
	if ((++fv_iter).is_valid()) {
		z = mesh_.vertex_handle(fv_iter->idx());
	}

	TriMesh::Point p = (mesh_.point(x) + mesh_.point(y) + mesh_.point(z)) / 3;
	w = mesh_.add_vertex(p);
	std::vector<int> idxs = mesh_.data(fh).index();

	mesh_.delete_face(fh);


	TriMesh::FaceHandle new_fh;
	new_fh = mesh_.add_face(x, y, w);
	reset_assignment(new_fh, idxs);
	new_fh = mesh_.add_face(y, z, w);
	reset_assignment(new_fh, idxs);
	new_fh = mesh_.add_face(z, x, w);
	reset_assignment(new_fh, idxs);

	return true;
}

double OptMeshSubdiv::face_tri_energy(TriMesh::FaceHandle fh)
{

	return mesh_.data(fh).face_energy;
}

double OptMeshSubdiv::face_tri_energy(TriMesh::HalfedgeHandle heh)
{
	return face_tri_energy(mesh_.face_handle(heh));
}

OptMeshSubdiv::OptMeshSubdiv()
{
}

OptMeshSubdiv::OptMeshSubdiv(TriMesh mesh)
{
	mesh_.clean();
	mesh_.clear();
	for (TriMesh::VertexIter v_it = mesh.vertices_begin();
		v_it != mesh.vertices_end(); ++v_it)
	{
		mesh_.add_vertex(mesh.point(*v_it));
		mesh_.data(*v_it).is_mesh_point_ = false;
	}
	mesh_.request_face_status();
	mesh_.request_edge_status();
	mesh_.request_vertex_status();
	threshold_ = 1.0;
	intialize_tetrahedron();
}


OptMeshSubdiv::~OptMeshSubdiv()
{
}

bool OptMeshSubdiv::SetThreshold(double threshold)
{
	threshold_ = threshold;
	return true;
}

bool OptMeshSubdiv::OneStepSubdivision()
{
	/*
		Assume updating vertices does not harm the projection assignment
	*/

	subdivide();
	vertexupdate();
	return true;
}

bool OptMeshSubdiv::GetResultMesh(TriMesh &mesh)
{
	mesh_.request_face_normals();
	mesh_.request_halfedge_normals();
	mesh_.request_vertex_normals();
	mesh_.update_normals();
	mesh = mesh_;
	return true;
}
