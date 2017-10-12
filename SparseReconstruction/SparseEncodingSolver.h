#pragma once
#include <vector>
#include <map>

#include "Triangle.h"
#include "TriMesh.h"

class SparseEncodingSolver
{
protected:
	//We will receive triangle list to replace B
	std::vector <TriProj::Triangle> encoded_B_;

	//We will return a mesh as a result
	//Also we need some method to construct 
	//matrix from mesh!
	void EdgeFlipUpdate();
	
	void BoundaryEdgeUpdate();
	


public:
	SparseEncodingSolver();
	~SparseEncodingSolver();
};

