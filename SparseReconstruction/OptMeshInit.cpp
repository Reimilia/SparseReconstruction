#include "OptMeshInit.h"
#include <cmath>


bool OptMeshInit::IsTriangleInMesh(
	TriMesh mesh, 
	TriProj::Triangle triangle,
	TriMesh::FaceHandle &fh_)
{
	int idx, idy, idz;

	triangle.GetTrianglePointIndex(idx, idy, idz);
	idx = sample_index[idx];
	idy = sample_index[idy];
	idz = sample_index[idz];
	TriMesh::VertexHandle X, Y, Z;
	X = mesh.vertex_handle(idx);
	Y = mesh.vertex_handle(idy);
	Z = mesh.vertex_handle(idz);
	//std::cout << triangle.RegEnergy() << ' ' << idx << ' ' << idy
	//	<< ' ' << idz << std::endl;

	//Check if this triangle already exists
	bool is_triangle_exists = false;
	TriMesh::HalfedgeHandle heh_ = mesh.find_halfedge(X, Y);

	if (heh_.is_valid() && mesh.opposite_vh(heh_) == Z)
	{
		fh_ = mesh.face_handle(heh_);
		return true;
	}
	heh_ = mesh.find_halfedge(Y, X);
	if (heh_.is_valid() && mesh.opposite_vh(heh_) == Z)
	{
		fh_ = mesh.face_handle(heh_);
		return true;
	}
	/*
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
			|| (aidx == idy && aidy == idx &&aidz == idz)
			|| (aidx == idy && aidy == idz &&aidz == idx)
			|| (aidx == idz && aidy == idx &&aidz == idy)
			|| (aidx == idz && aidy == idy &&aidz == idx)
			)
		{
			fh_ = mesh.face_handle(vf_iter->idx());
			is_triangle_exists = true;
			break;1
		}
	}*/
	
	
	return is_triangle_exists;
}

bool OptMeshInit::IsFeasible(TriMesh mesh, TriMesh::VertexHandle X, TriMesh::VertexHandle Y, TriMesh::VertexHandle Z)
{
	int isolated_num = 0;
	if (mesh.is_isolated(X)) isolated_num++;
	if (mesh.is_isolated(Y)) isolated_num++;
	if (mesh.is_isolated(Z)) isolated_num++;

	if (isolated_num==3)
		return true;
	//two isolated points

	//one isolated points
	
	//no isolated points
	bool is_boundary = (mesh.is_boundary(X) && mesh.is_boundary(Y) && mesh.is_boundary(Z));
	if (!is_boundary)
		return false;
	
	mesh.request_halfedge_status();
	mesh.request_vertex_status();
	mesh.request_face_status();
	TriMesh::FaceHandle f;
	bool flag = false;
	if (!mesh.find_halfedge(X, Y).is_valid()
		&& !mesh.find_halfedge(Y, Z).is_valid()
		&& !mesh.find_halfedge(Z, X).is_valid())
	{
		f = mesh.add_face(X, Y, Z);
		flag = (mesh.is_boundary(X) || mesh.is_manifold(X))
			&& (mesh.is_boundary(Y) || mesh.is_manifold(Y))
			&& (mesh.is_boundary(Z) || mesh.is_manifold(Z));
		mesh.delete_face(f, false);
		mesh.garbage_collection();
		if(flag) 
			return true;
		flag = mesh.find_halfedge(Y, X).is_valid()
			|| mesh.find_halfedge(Z, Y).is_valid()
			|| mesh.find_halfedge(X, Z).is_valid();
		if (flag)
			return true;
	}
	if (!mesh.find_halfedge(Y, X).is_valid()
		&& !mesh.find_halfedge(Z, Y).is_valid()
		&& !mesh.find_halfedge(X, Z).is_valid())
	{
		f = mesh.add_face(X, Z, Y);
		flag = (mesh.is_boundary(X) || mesh.is_manifold(X))
			&& (mesh.is_boundary(Y) || mesh.is_manifold(Y))
			&& (mesh.is_boundary(Z) || mesh.is_manifold(Z));
		mesh.delete_face(f, false);
		mesh.garbage_collection();
		if (flag)
			return true;
		flag = mesh.find_halfedge(X, Y).is_valid()
			|| mesh.find_halfedge(Y, Z).is_valid()
			|| mesh.find_halfedge(Z, X).is_valid();
		if (flag)
			return true;
	}
	return false;
}

void OptMeshInit::CWOrientation(TriMesh mesh, TriMesh::VertexHandle & X, TriMesh::VertexHandle & Y, TriMesh::VertexHandle & Z)
{
	// Change Vertex Handle into clockwise order
	
}


bool OptMeshInit::ManifoldCheck(TriMesh mesh, TriProj::Triangle triangle)
{
	/*
		1. Check if each point is ok
		2. Check if each edge is ok
	*/
	int idx, idy, idz;

	triangle.GetTrianglePointIndex(idx, idy, idz);
	idx = sample_index[idx];
	idy = sample_index[idy];
	idz = sample_index[idz];
	TriMesh::VertexHandle X, Y, Z;
	X = mesh.vertex_handle(idx);
	Y = mesh.vertex_handle(idy);
	Z = mesh.vertex_handle(idz);
	
	// Check if we have space to insert the triangle
	bool is_edge_fill_up = IsFeasible(mesh, X, Y, Z);
	if(!is_edge_fill_up)
		return false;
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
	sampler.SetRaduis(1/points.size());

	
	if (!sampler.GenerateSamples(mesh_size_, sample_index))
	{
		std::cerr << "Insufficient points!"
			"The sampling paramter is not setting properly.\n";
	}

	is_mesh_dict = new int[input_size_];

	for (int i = 0; i < points.size(); i++)
	{
		is_mesh_dict[i] = -1;
	}
	mesh_size_ = sample_index.size();

	for (int i = 0; i < mesh_size_; i++)
	{
		mesh_points_.push_back(points[sample_index[i]]);
		is_mesh_dict[sample_index[i]] = i;
	}
	for (int i = 0; i < points.size(); i++)
	{
		query_points_.push_back(points[i]);	
	}

	//Generate ANN kdtree
	int dim = 3;
	int point_set_size_ = (int)query_points_.size();

	ANNpointArray ANN_input_points;

	ANN_input_points = annAllocPts(point_set_size_, dim);

	for (int i = 0; i < point_set_size_; ++i)
	{

		for (int j = 0; j < dim; j++)
		{
			ANN_input_points[i][j] = query_points_[i][j];
		}
	}

	//build up kdTree
	kdtree_ = new ANNkd_tree(
		ANN_input_points,
		point_set_size_,
		dim
	);
	
	return true;
}


OptMeshInit::OptMeshInit()
{
	
}


OptMeshInit::~OptMeshInit()
{
	//Don't forget to delete variables.
	if (is_mesh_dict)
		delete[] is_mesh_dict;
	if (kdtree_)
		delete kdtree_;
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
	std::queue <int> queue_points_;
	bool *hash = new bool[query_points_.size()];
	for (int i = 0; i < query_points_.size(); i++)
	{
		//queue_points_.push(i);
		hash[i] = false;
		TriMesh::Point vertex(query_points_[i].data());
		TriMesh::VertexHandle vh_ = mesh.add_vertex(vertex);
		mesh.data(vh_).is_mesh_point_ = is_mesh_dict[i];
	}

	queue_points_.push(0);
	hash[0] = true;
	
	std::vector <TriProj::Triangle> triangles;

	while (!queue_points_.empty())
	{
		int i = queue_points_.front();
		queue_points_.pop();
		//if (!is_mesh_dict[i]) continue;
		std::cout << "point " << i << std::endl;
		int j = 0;
		triset.GenerateTriangleSet(
			query_points_[i],
			triangles
		);
		if (is_mesh_dict[i]!=-1)
		{
			std::cout << "Watch out!\n";
		}
		for (j = 0; j < triangles.size(); j++)
		{
			if (triangles[j].RegEnergy() == TriProj::inf)
			{
				j = triangles.size();
				break;
			}
			// Add triangle into mesh and test if this is the manifold
			TriMesh::FaceHandle fh_;
			if (IsTriangleInMesh(mesh, triangles[j], fh_))
			{
				sparse_encoding.push_back(triangles[j]);
				mesh.data(fh_).add_point_index(i);
				break;
			}

			//std::cout << triangles[j].RegEnergy() << std::endl;
			if (ManifoldCheck(mesh, triangles[j]))
			{
				int idx, idy, idz;

				triangles[j].GetTrianglePointIndex(idx, idy, idz);
				idx = sample_index[idx];
				idy = sample_index[idy];
				idz = sample_index[idz];
				std::cout << triangles[j].RegEnergy() << ' ' << idx << ' ' << idy
					<< ' ' << idz << std::endl;

				TriMesh::FaceHandle fh_ = mesh.add_face(
					mesh.vertex_handle(idx),
					mesh.vertex_handle(idy),
					mesh.vertex_handle(idz)
				);
				mesh.data(fh_).clear_point_index();
				mesh.data(fh_).add_point_index(i);
				sparse_encoding.push_back(triangles[j]);
				break;
			}

		}

		ANNidxArray		ANN_index;
		ANNdistArray	ANN_dist;
		ANNpoint		ANN_point;
		ANN_index = new ANNidx[10];
		ANN_dist = new ANNdist[10];
		ANN_point = annAllocPt(3);

		for (int k = 0; k < 3; k++)
			ANN_point[k] = query_points_[i][k];

		kdtree_->annkSearch(
			ANN_point,
			10,
			ANN_index,
			ANN_dist,
			0.0
		);

		for (int k = 0; k < 10; k++)
		{
			if (!hash[ANN_index[k]])
			{
				hash[ANN_index[k]] = true;
				queue_points_.push(ANN_index[k]);
			}
		}
		if (ANN_index)
			delete ANN_index;
		if (ANN_dist)
			delete ANN_dist;
		if (ANN_point)
			delete ANN_point;

		if (j >= triangles.size())
		{
			std::cout << "Error at" << i << ", No triangle will be paired "
				"with this point!" << std::endl;
			//queue_points_.push(i);
		}
		
	}

	
	bool is_manifold = true;
	for (TriMesh::VertexIter v_it = mesh.vertices_begin();
		v_it != mesh.vertices_end(); v_it++)
	{
		is_manifold &= mesh.is_manifold(*v_it);
	}
	if (!is_manifold)
		std::cout << "Opps!\n";
	
	return true;

}
