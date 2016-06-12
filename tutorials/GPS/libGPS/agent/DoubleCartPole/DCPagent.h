#ifndef DCPAGENT_H
#define DCPAGENT_H

#include <iostream>
#include <string>
#include "../agent.h"
#include "../../utility/utility.h"

namespace GPS_NSpace
{

using namespace std;

class DCPagent: public agent
{
public:
	DCPagent(hyperparameters& _mHyperparameters);
	void sample();
	void get_samples();
// --------------------------------------------------
//  DCPagent config
	bool rendering;

};

}

#endif
