#pragma once

#include <OpenMesh\Core\Mesh\PolyMesh_ArrayKernelT.hh>
#include <OpenMesh\Core\Mesh\PolyMeshT.hh>
#include <OpenMesh\Core\Mesh\TriMesh_ArrayKernelT.hh>
#include <OpenMesh\Core\Mesh\Traits.hh>

/*
This Program shall define the mesh we used in this program.
*/

struct MyTraits : public OpenMesh::DefaultTraits
{
  // use double valued coordinates
  typedef OpenMesh::Vec3d Point;
  // use vertex normals and vertex colors
  VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);
  // store the previous halfedge
  Half
  HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );
  // use face normals
  FaceAttributes( OpenMesh::Attributes::Normal );
  // store a face handle for each vertex
  VertexTraits
  {
    typename Base::Refs::FaceHandle my_face_handle;
  };
};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;

class DictMesh
{
public:
	DictMesh();
	~DictMesh();
};

