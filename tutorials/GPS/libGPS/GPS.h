#ifndef GPS_H
#define GPS_H

#include <iostream>
#include <memory>
#include "agent/agent.h"
#include "algorithm/algorithm.h"

namespace GPS
{

using namespace std;

class GPS
{
public:
	GPS();
	void run();
	void take_sample(int iter, int cond, int i);
	void take_iteration();
	void take_policy_samples();
// --------------------------------------------------
	int conditions;
	int train_idx; // python range
	int test_idx;  // python range
	int iterations;
	int num_samples;
// --------------------------------------------------
	unique_ptr<agent> mAgent;
	unique_ptr<algorithm> mAlgorithm;

};





}

#endif
