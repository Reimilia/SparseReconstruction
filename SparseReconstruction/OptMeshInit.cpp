#include "OptMeshInit.h"



bool OptMeshInit::IsTriangleInMesh(
	TriMesh mesh, 
	TriProj::Triangle triangle,
	TriMesh::FaceHandle &fh_)
{
	int idx, idy, idz;

	triangle.GetTrianglePointIndex(idx, idy, idz);
	TriMesh::VertexHandle X, Y, Z;
	X = mesh.vertex_handle(idx);
	Y = mesh.vertex_handle(idy);
	Z = mesh.vertex_handle(idz);
	//std::cout << std::endl << idx << ' ' << idy << ' ' << idz << ":\n";

	//Check if this triangle already exists
	bool is_triangle_exists = false;
	for (TriMesh::VertexFaceIter vf_iter = mesh.vf_iter(X);
		vf_iter.is_valid(); vf_iter++)
	{
		TriMesh::FaceVertexIter fv_iter = mesh.fv_iter(*vf_iter);
		int aidx, aidy = -1, aidz = -1;
		aidx = fv_iter->idx();
		if ((++fv_iter).is_valid()) aidy = fv_iter->idx();
		if ((++fv_iter).is_valid()) aidz = fv_iter->idx();
		//std::cout << aidx << ' ' << aidy << ' ' << aidz << std::endl;
		if ((aidx == idx && aidy == idy &&aidz == idz)
			|| (aidx == idx && aidz == idy &&aidy == idz)
			//|| (aidx == idy && aidy == idx &&aidz == idz)
			//|| (aidx == idy && aidy == idz &&aidz == idx)
			//|| (aidx == idz && aidy == idx &&aidz == idy)
			//|| (aidx == idz && aidy == idy &&aidz == idx)
			)
		{
			fh_ = mesh.face_handle(vf_iter->idx());
			is_triangle_exists = true;
			break;
		}
	}
	/*is_triangle_exists = (mesh.find_halfedge(X, Y) != TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Y, Z) != TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Z, X) != TriMesh::InvalidHalfedgeHandle)
		||
		(mesh.find_halfedge(X, Z) != TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Z, Y) != TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Y, X) != TriMesh::InvalidHalfedgeHandle);
		*/
	return is_triangle_exists;
}

bool OptMeshInit::IsEdgeFillUp(TriMesh mesh, TriMesh::VertexHandle X, TriMesh::VertexHandle Y)
{
	//Check halfedge status
	return (mesh.find_halfedge(X, Y) != TriMesh::InvalidHalfedgeHandle
		&&mesh.find_halfedge(Y, X) != TriMesh::InvalidHalfedgeHandle);
}

bool OptMeshInit::ManifoldCheck(TriMesh mesh, TriProj::Triangle triangle)
{
	/*
		1. Check if each point is ok
		2. Check if each edge is ok
	*/
	int idx, idy, idz;

	triangle.GetTrianglePointIndex(idx, idy, idz);
	TriMesh::VertexHandle X, Y, Z;
	X = mesh.vertex_handle(idx);
	Y = mesh.vertex_handle(idy);
	Z = mesh.vertex_handle(idz);
	
	// Check if we have space to insert the triangle
	bool is_edge_fill_up = (mesh.find_halfedge(X, Y) == TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Y, Z) == TriMesh::InvalidHalfedgeHandle
		&& mesh.find_halfedge(Z, X) == TriMesh::InvalidHalfedgeHandle)
		||
		(mesh.find_halfedge(X, Z) == TriMesh::InvalidHalfedgeHandle
			&& mesh.find_halfedge(Z, Y) == TriMesh::InvalidHalfedgeHandle
			&& mesh.find_halfedge(Y, X) == TriMesh::InvalidHalfedgeHandle);
	bool is_boundary = (mesh.is_boundary(X) && mesh.is_boundary(Y) && mesh.is_boundary(Z));
	if(!is_edge_fill_up || !is_boundary)
		return false;

	//if (!(mesh.is_boundary(X) && mesh.is_boundary(Y)&& mesh.is_boundary(Z)))
	//	return false;
	
	/*TriMesh::FaceHandle f = mesh.add_face(X, Y, Z);
	bool flag = true;
	mesh.request_vertex_status();
	mesh.request_edge_status();
	mesh.request_face_status();
	mesh.request_halfedge_status();
	for (TriMesh::FaceHalfedgeIter fh_it = mesh.fh_iter(f);
		fh_it.is_valid(); fh_it++)
	{
		if (!mesh.is_boundary(*fh_it))
		{
			if (!mesh.is_flip_ok(mesh.edge_handle(*fh_it)))
			{
				flag = false;
				break;
			}
		}
	}
	
	mesh.delete_face(f,false);
	mesh.garbage_collection();*/
	return true;
}

bool OptMeshInit::GenerateInitialDict(
	std::vector<Eigen::Vector3d> points)
{
	std::vector<double *> point_pool;
	for (int i = 0; i < points.size(); i++)
	{
		point_pool.push_back(points[i].data());
	}
	PoissonSampling sampler(point_pool);
	sampler.SetRaduis(0.05);

	std::vector<int> indexes;
	if (!sampler.GenerateSamples(mesh_size_, indexes))
	{
		std::cerr << "Insufficient points!"
			"The sampling paramter is not setting properly.\n";
	}

	bool *is_mesh_dict = new bool[input_size_];

	for (int i = 0; i < points.size(); i++)
	{
		is_mesh_dict[i] = false;
	}
	mesh_size_ = indexes.size();

	for (int i = 0; i < mesh_size_; i++)
	{
		mesh_points_.push_back(points[indexes[i]]);
		is_mesh_dict[indexes[i]] = true;
	}
	for (int i = 0; i < points.size(); i++)
	{
		if (!is_mesh_dict[i])
		{
			std::cout << i << std::endl;
			query_points_.push_back(points[i]);
		}
			
	}

	//Don't forget to delete variables.
	if(is_mesh_dict)
		delete[] is_mesh_dict;

	return true;
}


OptMeshInit::OptMeshInit()
{
	
}


OptMeshInit::~OptMeshInit()
{
}

bool OptMeshInit::BuildInitialSolution(
	OptSolverParaSet para,
	std::vector<Eigen::Vector3d> input_points,
	TriMesh & mesh,
	std::vector<TriProj::Triangle> & sparse_encoding)
{
	/*if (!mesh.vertices_empty())
	{
		std::cerr << "This is not an initial empty mesh!" << std::endl;
		return false;
	}*/

	mesh.clean();
	mesh.garbage_collection();
	input_size_ = input_points.size();
	mesh_size_ = round(para.initial_dict_ratio_*input_size_);

	//Step 1: Resampling
	if (!GenerateInitialDict(input_points))
	{
		std::cerr << "Something goes wrong when"
			"resampling from point clouds!" << std::endl;
		return false;
	}

	//Step 2: Generate Triangle Set
	//Step 3: Only Pick Those that pass manifold check
	/*
		The manifold check here cannot be fully considered
		Since you cannot judge the self-intersection between
		two triangles easily.
	*/
	TriProj::TriSet triset(
			para.triangle_set_control_num_,
			mesh_points_
		);

	//Add points in order
	for (int i = 0; i < mesh_points_.size(); i++)
	{
		TriMesh::Point vertex(mesh_points_[i].data());
		mesh.add_vertex(vertex);
	}
	
	std::vector <TriProj::Triangle> triangles;
	for (int i = 0; i < query_points_.size(); i++)
	{
		int j=0;
		triset.GenerateTriangleSet(
			query_points_[i],
			triangles
		);
		for (j = 0; j < triangles.size(); j++)
		{
			// Add triangle into mesh and test if this is the manifold
			TriMesh::FaceHandle fh_;
			if (IsTriangleInMesh(mesh, triangles[j], fh_))
			{
				sparse_encoding.push_back(triangles[j]);
				mesh.data(fh_).add_point_index(i);
				break;
			}
			if (ManifoldCheck(mesh, triangles[j]))
			{
				int idx, idy, idz;
				triangles[j].GetTrianglePointIndex(idx, idy, idz);
				TriMesh::FaceHandle fh_ = mesh.add_face(
					mesh.vertex_handle(idx),
					mesh.vertex_handle(idy),
					mesh.vertex_handle(idz)
				);
				mesh.data(fh_).add_point_index(i);
				sparse_encoding.push_back(triangles[j]);
				break;
			}

		}
		if (j >= triangles.size())
		{
			std::cout << "Error at" << i <<", No triangle will be paired "
			"with this point!"<< std::endl;
		}
	}
	/*
	bool is_manifold = true;
	for (TriMesh::VertexIter v_it = mesh.vertices_begin();
		v_it != mesh.vertices_end(); v_it++)
	{
		is_manifold &= mesh.is_manifold(*v_it);
	}
	if (!is_manifold)
		std::cout << "Opps!\n";
	*/
	return true;

}
