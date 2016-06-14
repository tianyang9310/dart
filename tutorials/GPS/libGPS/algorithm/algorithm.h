#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <iostream>
#include <memory>
#include "../agent/agent.h"

namespace GPS_NSpace
{

using namespace std;

class algorithm
{
public:
	algorithm();
	algorithm(bool placeholder);
	virtual void iteration();
	
	// cur
	// policy_opt.policy()

// --------------------------------------
//	algorithm config
	// auto algorithm_cost
	// auto algorithm_dynamics
	// auto algorithm_init_traj_distr
	double algorithm_initial_state_var;
	int algorithm_inner_iterations;
	double algorithm_kl_step;
	double algorithm_max_step_mult;
	double algorith_min_eta;
	double algorithm_min_step_mult;
	double algorithm_sample_decrease_var;
	double algorithm_sample_increase_var;
	// auto algorithm_trajopt
	

// --------------------------------------
//	algorithm member var
	int M;
	int _cond_idx;
	int iteration_count;
	shared_ptr<agent> algorithmAgent;

	int T;
	int dU;
	int dX;
	int dO;
	
	
};



}


#endif
