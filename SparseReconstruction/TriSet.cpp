#include "TriSet.h"

namespace TriProj
{
	TriSet::TriSet(int kNN_size, std::vector<Eigen::Vector3d> input_points)
	{
	}

	TriSet::~TriSet()
	{
		delete kdtree_;
	}

	bool TriSet::GenerateSparseEncoding(Eigen::Vector3d query_point, Eigen::Vector3i & triangle_set, Eigen::Triplet<double>& B_encoding)
	{
		return false;
	}


	bool TriSet::SetQuerySize(int k)
	{
		kNN_size_ = k;
		return true;
	}

}

