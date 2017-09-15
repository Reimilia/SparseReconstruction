#pragma once

/*
	For initialization process, we need a triangle set for each given point.

	This set will sort the triangle with designate energy function with respect
	to the point and triangle

*/

//TODO : Connect to ANN 
#include <Eigen\Core>


class TriSet
{
protected:
	
	int x;

public:
	TriSet();
	~TriSet();

	bool InitializeKNN(int KNN_number);

};

