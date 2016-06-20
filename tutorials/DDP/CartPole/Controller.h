#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include "dart/dart.h"
#include "../libDDP/DDP.h"
#include "CartPoleUtility.h"
#include <functional>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

using namespace CartPoleUtility;
using namespace Eigen;
using namespace std;
using namespace DDP_NSpace;

class Controller
{
public:
	Controller(WorldPtr mDDPWorld);
//------------------------------------------------------------------------------------------------
	std::unique_ptr<DDP> mDDP;
	Eigen::Vector4d x0;
};

#endif
