#pragma once

#include "TriSet.h"
namespace Test {
	class ANNtest
	{
	protected:
		int random_sample_num_ = 10;
		int kNN_size_ = 5;
		double random();

	public:
		ANNtest();

		bool TestTriSetANN();

		~ANNtest();
	};


}
/*
	Test the performance with ANN
*/
