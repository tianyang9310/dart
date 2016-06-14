#include "algorithm.h"
#include "../utility/utility.h"

namespace GPS_NSpace
{

extern hyperparameters mHyperparameters;

algorithm::algorithm()
{
	// placeholders doing nothing
}

algorithm::algorithm(bool placeholder)
{
	// update mHyparameters
	// auto algorithm_cost
	// auto algorithm_dynamics
	// auto algorithm_init_traj_distr
	algorithm_initial_state_var		= 1e-6;
	algorithm_inner_iterations		= 1;
	algorithm_kl_step				= 0.2;
	algorithm_max_step_mult			= 10.0;
	algorith_min_eta				= 1e-5;
	algorithm_min_step_mult			= 0.01;
	algorithm_sample_decrease_var	= 0.5;
	algorithm_sample_increase_var	= 1.0;
	// auto algorithm_trajopt

	//mHyperparameters.tupleauto
	//mHyperparameters.tupleauto
	//mHyperparameters.tupleauto
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_initial_state_var",algorithm_initial_state_var));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_inner_iterations",algorithm_inner_iterations));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_kl_step",algorithm_kl_step));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_max_step_mult",algorithm_max_step_mult));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorith_min_eta",algorith_min_eta));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_min_step_mult",algorithm_min_step_mult));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_sample_decrease_var",algorithm_sample_decrease_var));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_sample_increase_var",algorithm_sample_increase_var));
	//mHyperparameters.tupleauto
	
	M = mHyperparameters.conditions;
	_cond_idx = M; // python range
	iteration_count = 0;


	//grab a few value from the agent
	algorithmAgent = mHyperparameters.hyperparametersAgent;
	T=algorithmAgent->T;
	dU=algorithmAgent->dU;
	dX=algorithmAgent->dX;
	dO=algorithmAgent->dO;
}

void algorithm::iteration()
{
	cout<<"NoImplementationError"<<endl;
	throw 0;
}


}
