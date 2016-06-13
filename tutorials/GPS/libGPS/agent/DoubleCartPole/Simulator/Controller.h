#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "dart/dart.h"
#include <functional>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

using namespace Eigen;
using namespace std;

class Controller
{
public:
	Controller(WorldPtr mDDPWorld);
	Controller();
//------------------------------------------------------------------------------------------------

};

#endif
