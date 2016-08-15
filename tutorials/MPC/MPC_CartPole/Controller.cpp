#include "Controller.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

Controller::Controller(WorldPtr mDDPWorld)
{
	double m_c			= mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mCart_body")->getMass();
	double m_p			= mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mPole_end")->getMass();
	double l			= std::dynamic_pointer_cast<CylinderShape>(mDDPWorld->getSkeleton("mCartPole")->getBodyNode("mPole_body")->getCollisionShape(0))->getHeight();
	double g			= -mDDPWorld->getGravity()(2);;
	double delta_t		= mDDPWorld->getTimeStep();

	x0  = Eigen::Vector4d::Zero();
	x0(1)  = 1.0*M_PI;
	x0(3)  = 0.05*M_PI;


	Eigen::Vector4d	xd	= Eigen::Vector4d::Zero();
	xd(1)				= M_PI;

	Eigen::Matrix4d Q	= Eigen::Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;
	Q(2,2)				= 5;
	Q(3,3)				= 5;

	Eigen::Matrix<double,1,1> R;
	R(0,0)				= 1;

	Eigen::Matrix4d Qf = Eigen::Matrix4d::Identity();
	Qf(0,0)				= 10;
	Qf(1,1)				= 500;
	Qf(2,2)				= 500;
	Qf(3,3)				= 500;

	Q			= Q*delta_t;
	R			= R*delta_t;

//  Dynamics from DART
    mMPC = std::unique_ptr<MPC>(new MPC(10,100,4,1,x0,xd,Q,R,Qf, std::bind(DartStepDynamics, std::placeholders::_1, std::placeholders::_2, mDDPWorld), std::bind(CartPoleStepCost, std::placeholders::_1, std::placeholders::_2, xd, Q, R), std::bind(CartPoleFinalCost,std::placeholders::_1, xd, Qf)));

}
