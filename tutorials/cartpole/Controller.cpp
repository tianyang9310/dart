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

	Eigen::Vector4d x0;
	x0(0) =	mDDPWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition();
	x0(1) =	mDDPWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
	x0(2) =	mDDPWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity();
	x0(3) =	mDDPWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity();

	Eigen::Vector4d	xd	= Eigen::Vector4d::Zero();
	xd(1)				= M_PI;

	Eigen::Matrix4d Q	= Eigen::Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 0.01;

	Eigen::Matrix<double,1,1> R;
	R(0,0)				= 1;

	Eigen::Matrix4d Qf = Eigen::Matrix4d::Identity();
	Qf(1,1)				= 100;

	Q			= Q*delta_t;
	R			= R*delta_t;

	std::vector<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>> LQR;
	LQR.push_back(std::make_tuple(Q,R,Qf));

	

//	mDDP = std::unique_ptr<DDP>(new DDP(2000, std::bind(CartPoleStepDynamics, std::placeholders::_1, std::placeholders::_2, m_c, m_p, l, g, delta_t), std::bind(CartPoleStepCost, std::placeholders::_1, std::placeholders::_2, xd, Q, R), std::bind(CartPoleFinalCost,std::placeholders::_1, xd, Qf), LQR, std::make_tuple(x0,xd)));

	mDDP = std::unique_ptr<DDP>(new DDP(2000, std::bind(DartStepDynamics, std::placeholders::_1, std::placeholders::_2, mDDPWorld), std::bind(CartPoleStepCost, std::placeholders::_1, std::placeholders::_2, xd, Q, R), std::bind(CartPoleFinalCost,std::placeholders::_1, xd, Qf), LQR, std::make_tuple(x0,xd)));
}
