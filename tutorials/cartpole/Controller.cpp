#include "Controller.h"
#include "DDP.h"
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

extern double cart_mass;
extern double end_mass;
extern double pole_height;
extern double g;

Controller::Controller(SkeletonPtr mCartPole, double delta_t, WorldPtr mDDPWorld):mCartPole(mCartPole)
{
	mDDP = std::unique_ptr<DDP>(new DDP(2000, cart_mass, end_mass, pole_height, g, delta_t, mDDPWorld));
}
