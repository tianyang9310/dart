#include "Controller.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

Controller::Controller(WorldPtr mDDPWorld)
{
//	double m_c			= mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mCart_body")->getMass();
//	double m_p			= mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mPole_end")->getMass();
//	double l			= std::dynamic_pointer_cast<CylinderShape>(mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mPole_body")->getCollisionShape(0))->getHeight();
//	double g			= -mDDPWorld->getGravity()(2);;
//	double delta_t		= mDDPWorld->getTimeStep();
//	mDDP = std::unique_ptr<DDP>(new DDP(2000, mDDPWorld, std::bind(CartPoleStepDynamics, std::placeholders::_1, std::placeholders::_2, m_c, m_p, l, g, delta_t)));

	mDDP = std::unique_ptr<DDP>(new DDP(2000, mDDPWorld, std::bind(DartStepDynamics, std::placeholders::_1, std::placeholders::_2, mDDPWorld)));
}
