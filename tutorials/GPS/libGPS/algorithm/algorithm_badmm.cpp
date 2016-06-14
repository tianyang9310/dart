#include "algorithm_badmm.h"
#include "../utility/utility.h"

namespace GPS_NSpace
{

extern hyperparameters mHyperparameters;

algorithm_badmm::algorithm_badmm():algorithm(true)
{
	// update mHyperparameters
	algorithm_badmm_ent_reg_schedule	   = 0.0;
	algorithm_badmm_exp_step_decrease 	   = 0.5;
	algorithm_badmm_exp_step_increase 	   = 2.0;
	algorithm_badmm_exp_step_lower	  	   = 1.0;
	algorithm_badmm_exp_step_upper	  	   = 0.5;
	algorithm_badmm_fixed_lg_step	  	   = 0;
	algorithm_badmm_init_pol_wt		  	   = 0.01;
	algorithm_badmm_inner_iterations = 4;
	algorithm_badmm_lg_step_schedule  	   = 10.0;
	algorithm_badmm_max_policy_samples	   = 20;
	algorithm_badmm_policy_dual_rate  	   = 0.1;
	algorithm_badmm_policy_dual_rate_covar = 0.0;
	algorithm_badmm_policy_sample_mode     = "add";

	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_ent_reg_schedule",algorithm_badmm_ent_reg_schedule));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_exp_step_decrease",algorithm_badmm_exp_step_decrease));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_exp_step_increase",algorithm_badmm_exp_step_increase));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_exp_step_lower",algorithm_badmm_exp_step_lower));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_exp_step_upper",algorithm_badmm_exp_step_upper));
	mHyperparameters.tupleInt.insert(pair<string,int>("algorithm_badmm_fixed_lg_step",algorithm_badmm_fixed_lg_step));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_init_pol_wt",algorithm_badmm_init_pol_wt));
	mHyperparameters.tupleInt.insert(pair<string,int>("algorithm_badmm_inner_iterations",algorithm_badmm_inner_iterations));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_lg_step_schedule",algorithm_badmm_lg_step_schedule));
	mHyperparameters.tupleInt.insert(pair<string,int>("algorithm_badmm_max_policy_samples",algorithm_badmm_max_policy_samples));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_policy_dual_rate",algorithm_badmm_policy_dual_rate));
	mHyperparameters.tupleDouble.insert(pair<string,double>("algorithm_badmm_policy_dual_rate_covar",algorithm_badmm_policy_dual_rate_covar));
	mHyperparameters.tupleString.insert(pair<string,string>("algorithm_badmm_policy_sample_mode",algorithm_badmm_policy_sample_mode));

}

void algorithm_badmm::iteration()
{
	cout<<"algorithm_badmm iteration"<<endl;
}



}
