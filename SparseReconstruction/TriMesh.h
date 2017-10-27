#pragma once

#include <vector>

// Add library support for OpenMesh
#include <Eigen\Core>

#include <OpenMesh\Core\IO\MeshIO.hh>
#include <OpenMesh\Core\Mesh\Traits.hh>
#include <OpenMesh\Core\Mesh\TriMesh_ArrayKernelT.hh>

typedef  Eigen::Matrix3Xd Mat3Xd;

//Typedef traits here
struct MyTraits : public OpenMesh::DefaultTraits
{
	// use double valued coordinates
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;

	// use vertex normals and vertex colors
	VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color);

	// store the previous halfedge
	HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge
						| OpenMesh::Attributes::Status);
	// use face normals
	//FaceAttributes(OpenMesh::Attributes::Normal);

	// store a face handle for each vertex
	/*template <class Base, class Refs> struct VertexT : public Base
	{
		int some_additional_index;
	};*/

	EdgeTraits{
		double energy;
	};

	//This is the key for topological encoding update
	FaceTraits{
	private:
		//Correspondence point index
		std::vector<Point> proj_point_index_;
		
	public:
		double face_energy;
		bool is_calculated;

		void add_point_index(const Point index)
		{
			proj_point_index_.push_back(index);
		}
		void set_point_index(std::vector<Point> index_list)
		{
			clear_point_index();
			for (size_t i = 0; i < index_list.size(); i++)
			{
				add_point_index(index_list[i]);
			}
		}
		void clear_point_index()
		{
			proj_point_index_.clear();
		}

		const std::vector<Point>& index() const
		{
			return proj_point_index_;
		}
	};

};

//Define the mesh we use in this program
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  TriMesh;

