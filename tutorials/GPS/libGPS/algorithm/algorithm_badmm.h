#ifndef ALGORITHM_BADMM_H
#define ALGORITHM_BADMM_H

#include <iostream>
#include <string>
#include "algorithm.h"

namespace GPS_NSpace
{
using namespace std;

class algorithm_badmm : public algorithm
{
public:
	algorithm_badmm();
	void iteration();
// ---------------------------------------
//	algorithm_badmm config
	double algorithm_badmm_ent_reg_schedule;
	double algorithm_badmm_exp_step_decrease;
	double algorithm_badmm_exp_step_increase;
	double algorithm_badmm_exp_step_lower;
	double algorithm_badmm_exp_step_upper;
	int algorithm_badmm_fixed_lg_step;
	double algorithm_badmm_init_pol_wt;
	int algorithm_badmm_inner_iterations;
	double algorithm_badmm_lg_step_schedule;
	int algorithm_badmm_max_policy_samples;
	double algorithm_badmm_policy_dual_rate;
	double algorithm_badmm_policy_dual_rate_covar;
	string algorithm_badmm_policy_sample_mode;

};

}

#endif
