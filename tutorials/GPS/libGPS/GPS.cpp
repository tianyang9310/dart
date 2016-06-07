#include "GPS.h"

namespace GPS
{

GPS::GPS()
{
	conditions  = 4;
	train_idx   = 4; // python range
	test_idx    = 4; // python range
	iterations  = 10;
	num_samples = 5;

	// initialize agent and algorithm
	mAgent		= unique_ptr<agent>(new agent());
	mAlgorithm	= unique_ptr<algorithm>(new algorithm());
}

void GPS::run()
{
	for (int iter=0; iter<iterations; iter++)
	{
		for (int cond=0; cond<train_idx; cond++)
		{
			for (int i=0; i<num_samples; i++)
			{
				take_sample(iter, cond, i);
			}
		}
		for (int cond=0; cond<train_idx; cond++)
		{
		   	//traj_sample_lists = mAgent->get_samples(cond, -num_samples);
		}
		// take_iteration(itr, traj_sample_lists);
		// pol_sample_lists = take_policy_samples()
	}

}

void GPS::take_sample(int iter, int cond, int i)
{

}

void GPS::take_iteration()
{

}

void GPS::take_policy_samples()
{

}



}
