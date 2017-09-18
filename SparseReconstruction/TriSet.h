#pragma once

/*
	For initialization process, we need a triangle set for each given point.

	This set will sort the triangle with designate energy function with respect
	to the point and triangle

*/

//TODO : Connect to ANN 
#include <Eigen\Core>
#include <vector>


class TriSet
{
protected:
	
	int kNN_size_;

	bool is_initialized_ = false;
	

public:
	TriSet();
	TriSet(int kNN_size, std::vector<Eigen::Vector3d> input_points);
	~TriSet();

	bool InitializeKdTree(int kNN_size, std::vector<Eigen::Vector3d> input_points);


	bool GetTriangleTriplet(Eigen::Vector3d query_point, 
		std::vector<Eigen::Vector3i> &triangle_set);

	bool SetQuerySize(int k);

};


/*
void ImageWrapping::Filling()
{

// We only tolerate a small area's points to be used as fixup measure
// It will be a constant number
float radius_tolerate_range_ = sqrt(rows_*cols_) / 100 > 5.0 ? 5.0 : sqrt(rows_*cols_) / 100;

int total_number = 0;

ANNidxArray ANN_index_;
ANNdistArray ANN_dist_;
ANNkd_tree *kdTree;
ANNpointArray ANN_input_points_;
ANNpoint query_point_, added_point_;

int k = 5; // number of nearest neighbors
int dim = 2; // dimension
double eps = 0; // error bound

query_point_ = annAllocPt(dim);

ANN_index_ = new ANNidx[k]; // allocate near neigh indices
ANN_dist_ = new ANNdist[k]; // allocate near neighbor dists

for (int i = 0; i < rows_; i++)
	for (int j = 0; j < cols_; j++)
	{
		if (have_mapping_[i][j]) total_number++;
	}

ANN_input_points_ = annAllocPts(total_number, dim);
int t = 0;
for (int i = 0; i < rows_; i++)
	for (int j = 0; j < cols_; j++)
	{
		if (have_mapping_[i][j])
		{
			ANN_input_points_[t][0] = i;
			ANN_input_points_[t++][1] = j;
		}

	}

kdTree = new ANNkd_tree( // build search structure
	ANN_input_points_, // the data points
	total_number, // number of points
	dim); // dimension of space

Vector3f fix_color_;
int real_num_ = 0;

for (int i = 0; i < rows_; i++)
	for (int j = 0; j < cols_; j++)
	{
		if (have_mapping_[i][j]) continue;
		query_point_[0] = i;
		query_point_[1] = j;
		kdTree->annkSearch( // search
			query_point_, // query point
			k, // number of near neighbors
			ANN_index_, // nearest neighbors (returned)
			ANN_dist_, // distance (returned)
			eps); // error bound
		real_num_ = 0;
		fix_color_.setZero();

		for (t = 0; t < k; t++)
		{
			if (ANN_dist_[t] > radius_tolerate_range_) continue;
			Vector3f p;
			cv::Vec3b pp = target_.at<cv::Vec3b>(ANN_input_points_[ANN_index_[t]][0], ANN_input_points_[ANN_index_[t]][1]);
			p << pp[0], pp[1], pp[2];
			fix_color_ += p;
			//cout << ANN_input_points_[ANN_index_[t]][0] << ' ' << ANN_input_points_[ANN_index_[t]][1] << endl;
			real_num_++;
		}
		//cout << endl;
		if (real_num_ == 0) continue;

		fix_color_ /= real_num_;
		cv::Vec3b fix_color_cv_;
		fix_color_cv_[0] = (int)fix_color_[0];
		fix_color_cv_[1] = (int)fix_color_[1];
		fix_color_cv_[2] = (int)fix_color_[1];

		target_.at<cv::Vec3b>(i, j) = fix_color_cv_;
	}

delete[] ANN_index_; // clean things up
delete[] ANN_dist_;
delete kdTree;
annClose(); // done with ANN
}
*/