#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include <functional>
#include "dart/dart.h"
#include "CartPoleUtility.h"
#include "../../DDP/libDDP/DDP.h"
#include "../libMPC/MPC.h"

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
using namespace MPC_NSpace;

class Controller
{
public:
	Controller(WorldPtr mDDPWorld);
//------------------------------------------------------------------------------------------------
	std::unique_ptr<MPC> mMPC;
	Eigen::Vector4d x0;
};

#endif
