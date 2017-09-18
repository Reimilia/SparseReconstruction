#pragma once
// Add library support for OpenMesh
#include <Eigen\Core>
#include <OpenMesh\Core\Mesh\TriMesh_ArrayKernelT.hh>

typedef  Eigen::Matrix3Xd Mat3Xd;

//Typedef traits here
struct MyTraits : public OpenMesh::DefaultTraits
{
	// use double valued coordinates
	typedef OpenMesh::Vec3d Point;
	// use vertex normals and vertex colors
	VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);

	// store the previous halfedge
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
	// use face normals
	FaceAttributes(OpenMesh::Attributes::Normal);

	// store a face handle for each vertex
	template <class Base, class Refs> struct VertexT : public Base
	{
		int some_additional_index;
	};
};

//Define the mesh we use in this program
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  TriMesh;

