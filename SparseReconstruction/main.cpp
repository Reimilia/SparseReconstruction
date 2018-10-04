

//#include "TriangleProjection.h"
//#include "ANNtest.h"
//#include "SizeBalancedTree.h"
#include <iostream>
using namespace std;

#include "Triangle.h"


int main(int argc, char *argv[])
{
	/*SizeBalancedTree <__Element> T;

	T.insert(__Element(1.0, 1));
	T.insert(__Element(2.0, 1));
	T.insert(__Element(1.0, 2));
	T.insert(__Element(0.5, 1));
	T.insert(__Element(1.0, 3));

	for (int i = 0; i < T.size(); i++)
	{
		__Element A(T.select(i));
		std::cout << A.key() << ' ' << A.index() << std::endl;
	}

	std::cout << T.contains(__Element(1.0, 2)) << std::endl;
	std::cout << T.contains(__Element(2.0, 2)) << std::endl;

	T.remove(__Element(1.0, 2));
	for (int i = 0; i < T.size(); i++)
	{
		__Element A(T.select(i));
		std::cout << A.key() << ' ' << A.index() << std::endl;
	}*/
	TriProj::Triangle A = TriProj::Triangle(
		TriProj::Vec3d(0, 0, 0),
		TriProj::Vec3d(1, 0, 0),
		TriProj::Vec3d(0, 1, 0)
		);
	TriProj::Triangle B = TriProj::Triangle(
		TriProj::Vec3d(0, 0, 0),
		TriProj::Vec3d(0, 1, 0),
		TriProj::Vec3d(0, 0, 1)
	);

	std::cout << TriProj::IsTriTriIntersect(A, B) << std::endl;

	system("pause");
	return 0;
}