#include "OptMeshInit.h"



OptMeshInit::OptMeshInit()
{
}


OptMeshInit::~OptMeshInit()
{
}

bool OptMeshInit::BuildInitialSolution(OptSolverParaSet para, TriMesh & initial_mesh, std::vector<TriProj::Triangle>& sparse_encoding)
{
	energy_func_ = para.energy_func_;

	return false;
}
