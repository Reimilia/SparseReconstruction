#pragma once
#include "TriProjEnergy.h"
#include "Triangle.h"
/*
Super utility struct for the optimization problem.

This will be viewed as a complete part 

*/

struct OptSolverParaSet
{
	// edge reg term tradeoff
	double	edge_reg_weight_;
	bool	is_edge_reg_on_;

	// norm reg term tradeoff
	double	normal_reg_weight_;
	bool	is_normal_reg_on_;

	// q-norm 
	double	q_norm_;

	// resampling size for initialization
	// It's a ratio and we will approximately
	// pick those many.
	double	initial_dict_ratio_;

	// the k value for kNN search
	int		triangle_set_control_num_;

	// For iteration, steps and convergence tolerance
	double	epsilion_;
	int		max_iter_step_;

	//TODO: add Dictionary Update Coefficient here



	void SetDefaultPara()
	{
		edge_reg_weight_ = 2.5;
		is_edge_reg_on_ = true;

		normal_reg_weight_ = 0;
		is_normal_reg_on_ = false;

		q_norm_ = 0.5;

		initial_dict_ratio_ = 0.4;
		
		triangle_set_control_num_ = 5;

		epsilion_ = 0.001;

		max_iter_step_ = 10;

	};
};

