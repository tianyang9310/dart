#ifndef DCPAGENT_H
#define DCPAGENT_H

#include <iostream>
#include "../agent.h"

namespace GPS_NSpace
{

using namespace std;

class DCPagent: public agent
{
public:
	DCPagent();
	void sample();
	void get_samples();

};

}

#endif
