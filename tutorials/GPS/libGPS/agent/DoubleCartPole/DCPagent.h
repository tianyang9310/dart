#ifndef DCPAGENT_H
#define DCPAGENT_H

#include <iostream>
#include <string>
#include "../agent.h"
#include "../../utility/utility.h"

#include "dart/dart.h"

#include "Simulator/WorldSetup.h"
#include "Simulator/MyWindow.h"
#include "Simulator/Controller.h"


namespace GPS_NSpace
{

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

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
	
// --------------------------------------------------
//  dart world
	MyWindow window;

};

}

#endif
