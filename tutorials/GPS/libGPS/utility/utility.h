#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>

namespace GPS_NSpace{

using namespace std;
using namespace Eigen;

class hyperparameters
{
public:
	hyperparameters();
	// -------------------
	// agent
	string typeAgent;
	VectorXd target_state;
	// ##world
	vector<VectorXd> x0;
	int rk;
	double dt;
	int substeps;
	int conditions;
	// ##pos_body_idx
	// ##pos_body_offset
	int T;
	// ## sensor_dims
	// ## state_include
	// ##obs_include

	// -------------------
	// algorithm
	string typeAlgorithm;
	int iterations;
	VectorXd lg_setp_schedule;
	double policy_dual_rate;
	VectorXd ent_reg_schedule;
	int fixed_lg_step;
	double kl_step;
	double min_step_mult;
	double max_step_mult;
	double sample_decrease_var;
	double sample_increase_var;

	// ## init_traj_distr
	// ## cost
	// ##	action cost
	// ## 	state cost
	// ## dynamics
	// ##	dynamicsLRPrior
	// ## traj_opt
	// ## policy_opt
	// ##	caffe
	// ## policy_prior
	// ##	policyPriorGMM
	
	// -------------------
	// config
	int num_samples;
	int verbose_trials;
	int verbose_policy_trials;


};

}

#endif
