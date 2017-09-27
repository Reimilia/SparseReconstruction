#include "OptMeshInit.h"



void OptMeshInit::CallDownSampling()
{
}

bool OptMeshInit::ManifoldCheck(TriMesh mesh)
{
	return false;
}

bool OptMeshInit::GenerateInitialDict(std::vector<Eigen::Vector3d> points)
{
	return false;
}

bool OptMeshInit::GetInitSparseEncoding(TriProj::Triangle & encoding)
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

	std::vector <TriProj::Triangle> triangles;
	for (int i = 0; i < query_points_.size(); i++)
	{
		triset.GenerateTriangleSet(
			query_points_[i],
			triangles
		);
		int j = 0;

	}

	return true;

}
