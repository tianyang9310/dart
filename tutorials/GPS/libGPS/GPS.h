#ifndef GPS_H
#define GPS_H

// system header
#include <iostream>
#include <memory>

// agent header
#include "agent/agent.h"
#include "agent/DoubleCartPole/DCPagent.h"

// algorithm header
#include "algorithm/algorithm.h"
#include "algorithm/algorithm_badmm.h"

namespace GPS_NSpace
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
