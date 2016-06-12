#include "GPS.h"

namespace GPS_NSpace
{

GPS::GPS()
{
	hyperparameters mHyperparameters;
	cout<<mHyperparameters.target_state<<endl;



	_conditions  = 4;
	_train_idx   = 4; // python range
	_test_idx    = 4; // python range
	iterations  = 10;
	num_samples = 5;

	// initialize agent and algorithm
	mAgent		= unique_ptr<agent>(new DCPagent());
	mAlgorithm	= unique_ptr<algorithm>(new algorithm_badmm());
}

void GPS::run()
{
	for (int iter=0; iter<iterations; iter++)
	{
		for (int cond=0; cond<_train_idx; cond++)
		{
			for (int i=0; i<num_samples; i++)
			{
				take_sample(iter, cond, i);
			}
		}
		for (int cond=0; cond<_train_idx; cond++)
		{
		   	//auto traj_sample_lists = mAgent->get_samples(cond, -num_samples);
		}
		// take_iteration(itr, traj_sample_lists);
		// pol_sample_lists = take_policy_samples()
	}

}

void GPS::take_sample(int iter, int cond, int i)
{
	// pol = mAlgorithm->cur(cond)->traj_distr;
	// mAgent->sample(pol,cond,verbose=False);
}

void GPS::take_iteration(/*int itr, auto traj_sample_lists*/)
{
	// mAlgorithm->iteartion(traj_sample_lists);
}

void GPS::take_policy_samples()
{
	// int N = verbose_policy_trials;
	// pol_samples = placeholders, matrix of samples
	for (int cond=0; cond<_test_idx; cond++)
	{
		for (int i=0; /*i<N*/; i++)
		{
			// pol_samples[cond][i] = mAgent->sample(mAlgorithm->policy_opt.policy, cond)
		}
	}
	// return SampleLists(samples) for samples in pol_samples
}



}
