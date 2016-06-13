#ifndef DCPAGENT_H
#define DCPAGENT_H

#include <iostream>
#include <string>
#include "../agent.h"
#include "../../utility/utility.h"

#include "dart/dart.h"

namespace GPS_NSpace
{

using namespace std;


class DCPagent: public agent
{
public:
	DCPagent();
	void sample();
	void get_samples();
	void _setup_world();
// --------------------------------------------------
//  DCPagent config
	bool rendering;

};

}

#endif
