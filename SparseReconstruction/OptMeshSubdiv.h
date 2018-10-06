#pragma once

#include <Eigen\Core>

#include "Triangle.h"
#include "TriMesh.h"
#include "DictionaryUpdate.h"
#include "SparseEncodingSolver.h"
// An alternative way to get through all the stuff
/*


1.	Get a tetrahedron (0,0,0) (1,0,0), (0,1,0), (0,0,1)
2.	Then it will recursively subdivide the face
3.	Optimize V and B if necessary.
*/


/*
	Temporially try to update only V first.
*/

class OptMeshSubdiv
{
protected:
	// control when to apply subdivision
	double		threshold_;
	TriMesh		mesh_;


private:
	// Very stupid method to obtain topological correctness
	bool intialize_tetrahedron();
	// Assign in one face with three coordinates
	bool reset_assignment(TriMesh::FaceHandle fh, std::vector<int> idxs);

protected:
	bool subdivide();
	bool topologyupdate();
	bool vertexupdate();
	bool assign_points(TriMesh::FaceHandle fh);
	double face_tri_energy(TriMesh::FaceHandle fh);
	double face_tri_energy(TriMesh::HalfedgeHandle fh);


public:
	OptMeshSubdiv();
	OptMeshSubdiv(TriMesh mesh);
	~OptMeshSubdiv();

	bool SetThreshold(double threshold_);

	bool OneStepSubdivision();
	bool OptMeshSubdiv::GetResultMesh(TriMesh &mesh);
	
};

