#pragma once
#include <OpenMesh\Core\Mesh\PolyMesh_ArrayKernelT.hh>
#include <OpenMesh\Core\Mesh\PolyMeshT.hh>
#include <OpenMesh\Core\Mesh\Attributes.hh>

/*
This Program shall define the mesh we used in this program.
*/

struct MyTraits : public OpenMesh::DefaultTraits
{
	//Customized Traits/Properties
	VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
	FaceAttributes(OpenMesh::Attributes::Normal);
	
};


typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits>  MyMesh;

class DictMesh
{
public:
	DictMesh();
	~DictMesh();
};

