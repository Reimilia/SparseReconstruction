#include "OptMeshInit.h"



void OptMeshInit::CallDownSampling()
{
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
	
	TriMesh::FaceHandle added_face= mesh.add_face(
		X,Y,Z
	);

	bool flag = mesh.is_manifold(X)
		&&mesh.is_manifold(Y)
		&&mesh.is_manifold(Z)
		&&((	
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(X,Y)))&&
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(Y, Z)))&&
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(Z, X)))

		)||(
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(Y, X))) &&
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(Z, Y))) &&
			mesh.is_simple_link(mesh.edge_handle(mesh.find_halfedge(X, Z)))

		));


	mesh.delete_face(added_face);
	mesh.garbage_collection();

	return false;
}

bool OptMeshInit::GenerateInitialDict(std::vector<Eigen::Vector3d> points)
{
	return false;
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
	if (!mesh.vertices_empty())
	{
		std::cerr << "This is not an initial empty mesh!" << std::endl;
		return false;
	}

	mesh.clean();
	mesh.garbage_collection();

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
		triset.GenerateTriangleSet(
			query_points_[i],
			triangles
		);
		for (int j = 0; j < triangles.size(); j++)
		{
			// Add triangle into mesh and test if this is the manifold
			if (ManifoldCheck(mesh, triangles[j]))
			{
				int idx, idy, idz;
				triangles[j].GetTrianglePointIndex(idx, idy, idz);
				mesh.add_face(
					mesh.vertex_handle(idx),
					mesh.vertex_handle(idy),
					mesh.vertex_handle(idz)
				);
				sparse_encoding.push_back(triangles[j]);
				break;
			}

		}
	}

	return true;

}
