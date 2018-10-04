#include "OptMeshInit.h"
#include <cmath>


OptMeshInit::OptMeshInit()
{
	has_mesh_created_ = false;
	mesh_.clear();
}


OptMeshInit::~OptMeshInit()
{
	//Don't forget to delete variables.
	if (is_mesh_dict)
		delete[] is_mesh_dict;
	if (kdtree_)
		delete kdtree_;
	if (hash_)
		delete hash_;
}

void OptMeshInit::GetResultMesh(TriMesh &out_mesh)
{
	if (!has_mesh_created_)
	{
		mesh_.request_face_normals();
		mesh_.request_halfedge_normals();
		mesh_.request_vertex_normals();
		mesh_.update_normals();
		has_mesh_created_ = true;
		bool is_manifold = true;
		for (TriMesh::VertexIter v_it = mesh_.vertices_begin();
			v_it != mesh_.vertices_end(); v_it++)
		{
			is_manifold &= (mesh_.is_manifold(*v_it) || mesh_.is_boundary(*v_it));
		}
		if (!is_manifold)
			std::cout << "Opps! Non-manifold in topology\n";
	}
	out_mesh = mesh_;
}

bool OptMeshInit::PairOneQueryPoint()
{
	//If queue is empty, we cannot push it back
	if (queue_points_.empty())
		return false;

	//Clear status
	if (has_mesh_created_)
		has_mesh_created_ = false;

	int i = queue_points_.front();

	std::vector <TriProj::Triangle> triangles;

	queue_points_.pop();
	//if (!is_mesh_dict[i]) continue;
	std::cout << "point " << i << std::endl;
	if (is_mesh_dict[i] != -1)
	{
		std::cout << "Watch out!\n";
		//return true;
	}

	int j = 0;
	
	if (para_.is_normal_reg_on_)
	{
		triset_.GenerateTriangleSet(
			query_points_[i],
			query_normals_[i],
			triangles
		);
	}
	else
	{
		triset_.GenerateTriangleSet(
			query_points_[i],
			triangles
		);

	}

	bool flag = false;
	
	for (j = 0; j < triangles.size(); j++)
	{
		int idx, idy, idz;
		triangles[j].GetTrianglePointIndex(idx, idy, idz);
		idx = sample_index[idx];
		idy = sample_index[idy];
		idz = sample_index[idz];
		//std::cout << triangles[j].RegEnergy() << ' ' << idx << ' ' << idy
		//	<< ' ' << idz << std::endl;
		if (triangles[j].RegEnergy() == TriProj::inf)
		{
			j = triangles.size();
			break;
		}
		mesh_.request_halfedge_status();
		mesh_.request_vertex_status();
		mesh_.request_face_status();
		// Add triangle into mesh and test if this is the manifold
		TriMesh::FaceHandle fh_;
		if (IsTriangleInMesh(triangles[j], fh_))
		{
			std::cout << "Triangle already exists!\n";
			std::cout << triangles[j].RegEnergy() << ' ' << idx << ' ' << idy
				<< ' ' << idz << std::endl;
			mesh_.data(fh_).add_point_index(i);
			flag = true; 
			break;
		}
		else
		{
			fh_ = ManifoldCheck(triangles[j]);
			if (fh_.is_valid())
			{
				mesh_.data(fh_).clear_point_index();
				mesh_.data(fh_).add_point_index(i);
				break;
			}
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
		if (!hash_[ANN_index[k]])
		{
			hash_[ANN_index[k]] = true;
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
	return true;
}


bool OptMeshInit::IsTriangleInMesh(
	TriProj::Triangle triangle,
	TriMesh::FaceHandle &fh_)
{
	int idx, idy, idz;

	triangle.GetTrianglePointIndex(idx, idy, idz);
	idx = sample_index[idx];
	idy = sample_index[idy];
	idz = sample_index[idz];
	TriMesh::VertexHandle X, Y, Z;
	X = mesh_.vertex_handle(idx);
	Y = mesh_.vertex_handle(idy);
	Z = mesh_.vertex_handle(idz);
	//std::cout << triangle.RegEnergy() << ' ' << idx << ' ' << idy
	//	<< ' ' << idz << std::endl;

	//Check if this triangle already exists
	bool is_triangle_exists = false;
	TriMesh::HalfedgeHandle heh_ = mesh_.find_halfedge(X, Y);

	if (heh_.is_valid() && mesh_.opposite_vh(heh_) == Z)
	{
		fh_ = mesh_.face_handle(heh_);
		return true;
	}
	heh_ = mesh_.find_halfedge(Y, X);
	if (heh_.is_valid() && mesh_.opposite_vh(heh_) == Z)
	{
		fh_ = mesh_.face_handle(heh_);
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
	
	
	return false;
}

TriMesh::FaceHandle OptMeshInit::IsFeasible(TriMesh::VertexHandle X, TriMesh::VertexHandle Y, TriMesh::VertexHandle Z)
{
	//special case, three points are brand new.
	
	int isolated_num = mesh_.is_isolated(X)+mesh_.is_isolated(Y)+mesh_.is_isolated(Z);
	if (isolated_num==3)
	{
		TriMesh::FaceHandle f = mesh_.add_face(X, Y, Z);
		return f;
	}

	bool is_not_isolated = (!mesh_.is_isolated(X) && !mesh_.is_isolated(Y) && !mesh_.is_isolated(Z));
		
	bool is_boundary = (mesh_.is_boundary(X) && mesh_.is_boundary(Y) && mesh_.is_boundary(Z));
	if (!is_boundary)
		return TriMesh::InvalidFaceHandle;

	TriMesh::FaceHandle f;
	TriMesh::HalfedgeHandle h1, h2, h3, h1o, h2o, h3o;
	h1 = mesh_.find_halfedge(X, Y);
	h2 = mesh_.find_halfedge(Y, Z);
	h3 = mesh_.find_halfedge(Z, X);

	h1o = mesh_.find_halfedge(Y, X);
	h2o = mesh_.find_halfedge(Z, Y);
	h3o = mesh_.find_halfedge(X, Z);

	bool flag = false;
	//If all three edges are available?
	if ( (h1.is_valid() && mesh_.is_boundary(h1)) || !h1.is_valid())
		if ( (h2.is_valid() && mesh_.is_boundary(h2)) || !h2.is_valid())
			if ( (h3.is_valid() && mesh_.is_boundary(h3)) || !h3.is_valid())
	{
		//reject the case when it does not share an edge with some existing triangles
		//If there is one isolated vertex
		//And reject the case where it does not share with two edges if 
		// no isolated vertices
		int validity_num = h1.is_valid() + h2.is_valid() + h3.is_valid();
		if (validity_num == 0)
			return TriMesh::InvalidFaceHandle;
		if (isolated_num == 0 && validity_num < 2)
			return TriMesh::InvalidFaceHandle;
		f = mesh_.add_face(X, Y, Z);
		if (!f.is_valid())
		{
			return TriMesh::InvalidFaceHandle;
		}

		// If adding this face will not cause manifold violation
		std::cout << "Add face passed\n";
		return f;
		/*flag = (mesh_.is_manifold(X) && mesh_.is_manifold(Y) && mesh_.is_manifold(Z));
		
		if (flag)
			return f;
		mesh_.delete_face(f, false);
		mesh_.garbage_collection();*/
	}
	/*if ((h1o.is_valid() && mesh_.is_boundary(h1o)) || !h1o.is_valid())
		if ((h2o.is_valid() && mesh_.is_boundary(h2o)) || !h2o.is_valid())
			if ((h3o.is_valid() && mesh_.is_boundary(h3o)) || !h3o.is_valid())
	{
		int validity_num = h1.is_valid() + h2.is_valid() + h3.is_valid();
		if (validity_num == 0)
			return TriMesh::InvalidFaceHandle;
		if (isolated_num == 0 && validity_num < 2)
			return TriMesh::InvalidFaceHandle;
		// If opposite halfedges are outer boundary(possible to add face)
		std::cout << "Pass2!\n";
		f = mesh_.add_face(X, Z, Y);
		if (!f.is_valid())
		{
			return TriMesh::InvalidFaceHandle;
		}
		//flag = (mesh_.is_manifold(X) && mesh_.is_manifold(Y) && mesh_.is_manifold(Z));
		std::cout << "Pass2.1\n";
		if (flag)
			return f;

		mesh_.delete_face(f, false);
		mesh_.garbage_collection();
		return f;
	}*/
	return TriMesh::InvalidFaceHandle;
}


TriMesh::FaceHandle OptMeshInit::ManifoldCheck(TriProj::Triangle triangle)
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
	TriMesh::Point PX, PY, PZ;
	X = mesh_.vertex_handle(idx);
	Y = mesh_.vertex_handle(idy);
	Z = mesh_.vertex_handle(idz);

	PX = mesh_.point(X);
	PY = mesh_.point(Y);
	PZ = mesh_.point(Z);

	std::cout << idx << ' ' << idy << ' ' << idz << std::endl;
	// Check if we have space to insert the triangle
	
	for (TriMesh::FaceIter fiter = mesh_.faces_begin(); fiter != mesh_.faces_end(); ++fiter)
	{
		TriMesh::FaceVertexIter fv_iter = mesh_.fv_iter(*fiter);
		TriMesh::Point x, y, z;
		x = mesh_.point(*fv_iter);
		if ((++fv_iter).is_valid()) y = mesh_.point(*fv_iter);
		if ((++fv_iter).is_valid()) z = mesh_.point(*fv_iter);
		bool tag = TriProj::IsTriTriIntersect(
			TriProj::Triangle(
				TriProj::Vec3d(x.data()),
				TriProj::Vec3d(y.data()),
				TriProj::Vec3d(z.data())
			),
			TriProj::Triangle(
				TriProj::Vec3d(PX.data()),
				TriProj::Vec3d(PY.data()),
				TriProj::Vec3d(PZ.data())
			)
		);
		if (tag)
			return TriMesh::InvalidFaceHandle;
	}

	return IsFeasible(X, Y, Z);
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
	sampler.SetRaduis(1.0/mesh_size_);

	
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


bool OptMeshInit::BuildInitialSolution(
	OptSolverParaSet para,
	TriMesh input_mesh)
{
	para_ = para;
	for (int i = 0; i < input_mesh.n_vertices(); i++)
	{
		TriMesh::Point p = input_mesh.point(input_mesh.vertex_handle(i));
		query_points_.push_back(Eigen::Vector3d(p.data()));
		if (para.is_normal_reg_on_)
		{
			TriMesh::Normal n = input_mesh.normal(input_mesh.vertex_handle(i));
			query_normals_.push_back(Eigen::Vector3d(n.data()));
		}
		
	}

	mesh_.clean();
	mesh_.garbage_collection();
	input_size_ = query_points_.size();
	mesh_size_ = round(para.initial_dict_ratio_*input_size_);

	//Step 1: Resampling
	if (!GenerateInitialDict(query_points_))
	{
		std::cerr << "Something goes wrong when"
			"resampling from point clouds!" << std::endl;
		return false;
	}

	//Step 2: Generate Triangle Set
	//Step 3: Only Pick Those that pass manifold check
	/*
		The manifold check here is not complete since no self-intersection
		has been made
	*/
	triset_.SetQuerySize(para.triangle_set_control_num_);
	triset_.SetInputPoints(mesh_points_);

	hash_ = new bool[query_points_.size()];
	//Add points in order
	for (int i = 0; i < query_points_.size(); i++)
	{
		//queue_points_.push(i);
		hash_[i] = false;
		
		TriMesh::Point vertex(query_points_[i].data());
		TriMesh::VertexHandle vh_ = mesh_.add_vertex(vertex);
		mesh_.data(vh_).is_mesh_point_ = is_mesh_dict[i];

		//Copy normal info.
		mesh_.set_normal(vh_, input_mesh.normal(input_mesh.vertex_handle(i)));
	}
	int k = 0;
	while (is_mesh_dict[k] != -1) k++;
	queue_points_.push(k);
	hash_[k] = true;
	while (PairOneQueryPoint()) ;
	return true;
}

bool OptMeshInit::TestPossionDiskSampling(
	OptSolverParaSet para,
	std::vector<Eigen::Vector3d> input_points, 
	TriMesh & initial_mesh
)
{
	initial_mesh.clean();
	initial_mesh.garbage_collection();
	input_size_ = input_points.size();
	mesh_size_ = round(para.initial_dict_ratio_*input_size_);

	if (!GenerateInitialDict(input_points))
	{
		std::cerr << "Something goes wrong when"
			"resampling from point clouds!" << std::endl;
		return false;
	}


	for (int i = 0; i < mesh_points_.size(); i++)
	{
		//queue_points_.push(i);
		TriMesh::Point vertex(mesh_points_[i].data());
		TriMesh::VertexHandle vh_ = initial_mesh.add_vertex(vertex);
	}
	return true;
}
