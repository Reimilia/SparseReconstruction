#include "DicUpdateTest.h"


DicUpdateTest::DicUpdateTest(TriMesh _mesh_)
{
	mesh_ = _mesh_;
}

DicUpdateTest::~DicUpdateTest()
{

}

TriMesh	DicUpdateTest::solver()
{
	compute();
	DictionaryUpdate DicUp(mesh_, V_, B_, P_);
	return DicUp.solver();
}

void DicUpdateTest::compute()
{
	// set up V_
	int wid_V = mesh_.n_vertices();
	V_.resize(3, wid_V);
	V_.setZero();
	
	for (auto v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); v_it++)
	{
		TriMesh::Point v_pos = mesh_.point(*v_it);
		V_.col(v_it->idx()) << v_pos[0], v_pos[1], v_pos[2];
	}

	// set up B_ and P_, sample 5 points for every face
	int wid_P = 5 * mesh_.n_faces();
	P_.resize(3, wid_P);
	
	int P_idx = 0;

	for (auto f_it = mesh_.faces_begin(); f_it != mesh_.faces_begin(); f_it++)
	{
		TriMesh::Normal f_normal = mesh_.calc_face_normal(*f_it);
		for (int k = 0; k < 5; k++) // sample 5 points of every face
		{
			std::vector<TriMesh::FaceVertexIter> fv_list = {};
			for (TriMesh::FaceVertexIter fv_it = mesh_.fv_begin(*f_it); fv_it != mesh_.fv_end(*f_it); fv_it++)
			{
				
				fv_list.push_back(fv_it);
			}

			srand((unsigned)time(NULL));
			double alpha = rand();
			double beta = rand();
			double gamma = rand();
			double sum = alpha + beta + gamma;
			alpha /= sum;
			beta /= sum;
			gamma /= sum;

			B_.insert(fv_list[0]->idx(), P_idx) = alpha;
			B_.insert(fv_list[1]->idx(), P_idx) = beta;
			B_.insert(fv_list[2]->idx(), P_idx) = gamma;

			TriMesh::Point point_P = alpha * mesh_.point(*fv_list[0]) + beta * mesh_.point(*fv_list[1]) + gamma * mesh_.point(*fv_list[2]) + double(rand()) / double(RAND_MAX) * f_normal;
			P_.col(P_idx) << point_P[0], point_P[1], point_P[2];

			P_idx++;
		}
	}
	B_.resize(wid_V, wid_P);
}