#include "ANNtest.h"
#include "TriProjEnergy.h"
#include <iostream>
#include <cstdlib>
#include <ctime>


double Test::ANNtest::random()
{
	return (rand() % 10000) / 10000.0;
}

Test::ANNtest::ANNtest()
{
}

bool Test::ANNtest::TestTriSetANN()
{
	srand((int)time(0));

	std::cout << "Test with " << random_sample_num_ << " generated samples.\n";
	std::cout << "KNN number is set to " << kNN_size_ << std::endl;

	std::vector <Eigen::Vector3d> q;

	for (int i = 1; i < random_sample_num_; i++)
	{
		Eigen::Vector3d p;
		p[0] = random();
		p[1] = random();
		p[2] = random();
		q.push_back(p);
	}

	std::cout << "\nRun Test!\n";

	TriProj::TriSet  *triset = new TriProj::TriSet(kNN_size_, q);
	triset->SetQuerySize(5);

	std::function<double(TriProj::Triangle)> f = 
		std::bind(EnergyFunc::EnergyWithEdgeNorm,
		std::placeholders::_1, 0.5, 1.6);


	std::vector<TriProj::Triangle> tri_index;

	triset->GenerateTriangleSet(Eigen::Vector3d(0.5, 0.5, 0.5), tri_index);

	std::cout << "The closest triangles are:\n";

	for (int i = 0; i < tri_index.size(); i++)
	{
		Eigen::Vector3d X, Y, Z;
		int idx, idy, idz;
		tri_index[i].GetTriangle(X, Y, Z);
		tri_index[i].GetTrianglePointIndex(idx, idy, idz);
		std::cout << idx << ':' << X.transpose() << std::endl
			<< idy << ":" << Y.transpose() << std::endl
			<< idz << ":" << Z.transpose() << std::endl ;
		std::cout << "With energy " << f(tri_index[i]) << std::endl << std::endl;
	}
	
	std::cout << "Test is over";
	delete triset;

	system("pause");
	return true;
}


Test::ANNtest::~ANNtest()
{
}
