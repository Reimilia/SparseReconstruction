#pragma once

#include<Eigen\Core>

class EnergyFunc
{

public:
	EnergyFunc();
	virtual double CalcEnergy();
	virtual ~EnergyFunc();
};

