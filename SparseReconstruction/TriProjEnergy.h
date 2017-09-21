#pragma once

#include "Triangle.h"

class TriProjEnergy
{

public:
	TriProjEnergy();
	virtual double CalcEnergy(TriProj::Triangle);
	virtual ~TriProjEnergy();
};

