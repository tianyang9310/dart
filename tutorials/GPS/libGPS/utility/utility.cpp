#include "utility.h"

namespace GPS_NSpace
{

hyperparameters::hyperparameters()	
{
	// -------------------
	// sensor dim
	SensorDim["action"]=1;
	SensorDim["state"]=6;
	SensorDim["observation"]=6;
	
	map<string,int> SensorDim;
	// -------------------
	// agent
	typeAgent = "DoubleCartPole";
	target_state.resize(6);
	target_state<<0,M_PI,0,0,0,0;
	// ##world
	
	for_each(x0.begin(), x0.end(),[](VectorXd& x0_sub){x0_sub.resize(6);});
	x0.push_back((VectorXd(6)<<0,0.2*M_PI,0.1*M_PI,0,0,0).finished());
	x0.push_back((VectorXd(6)<<0,0.1*M_PI,-0.3*M_PI,0,0,0).finished());
	x0.push_back((VectorXd(6)<<0,0.23*M_PI,0.13*M_PI,0,0,0).finished());
	x0.push_back((VectorXd(6)<<0,0.05*M_PI,0.28*M_PI,0,0,0).finished());

	rk=0;
	dt=0.05;
	substeps=1;
	conditions = 4;
	// ##pos_body_idx
	// ##pos_body_offset
	T = 100;
	// ## sensor_dims
	// ## state_include
	// ##obs_include
	

	// -------------------
	// algorithm
	typeAlgorithm = "AlgorithmBADMM";
	iterations = 10;
	lg_setp_schedule.resize(4);
	lg_setp_schedule<<1e-4,1e-3,1e-2,1e-1;
	policy_dual_rate = 0.2;
	ent_reg_schedule.resize(4);
	ent_reg_schedule<<1e-3,1e-3,1e-2,1e-1;
	fixed_lg_step = 3;
	kl_step = 5.0;
	min_step_mult = 0.01;
	max_step_mult = 1.0;
	sample_decrease_var = 0.05;
	sample_increase_var	= 0.1;
	

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
	num_samples = 5;
	verbose_trials = 0;
	verbose_policy_trials = 1;

}

}
