#include "Controller.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::common;
using namespace dart::math;
using namespace dart::gui;

Controller::Controller(WorldPtr mDDPWorld)
{
	double delta_t		= mDDPWorld->getTimeStep();

	x0.resize(6);
	x0.setZero();
//	x0(1)  = 0.1*M_PI;


	Eigen::VectorXd	xd(6);
	xd.setZero();
	xd(1)				= M_PI;

	Eigen::MatrixXd Q(6,6);
	Q.setZero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;
	Q(2,2)				= 5;
	Q(3,3)				= 8;
	Q(4,4)				= 8;
	Q(5,5)				= 8;

	Eigen::Matrix<double,1,1> R;
	R(0,0)				= 1;

	Eigen::MatrixXd Qf(6,6);
	Qf.setZero();
	Qf(0,0)				= 10;
	Qf(1,1)				= 500;
	Qf(2,2)				= 500;
	Qf(3,3)				= 800;
	Qf(4,4)				= 800;
	Qf(5,5)				= 800;

	Q			= Q*delta_t;
	R			= R*delta_t;

	std::vector<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>> LQR;

// --------------------------------------------------
//  Linear Quadratic Cost function
	LQR.push_back(std::make_tuple(Q,R,Qf));

//  Dynamics from DART
	mDDP = std::unique_ptr<DDP>(new DDP(2000, std::bind(DartStepDynamics, std::placeholders::_1, std::placeholders::_2, mDDPWorld), std::bind(CartPoleStepCost, std::placeholders::_1, std::placeholders::_2, xd, Q, R), std::bind(CartPoleFinalCost,std::placeholders::_1, xd, Qf), LQR, std::make_tuple(x0,xd,1)));


// --------------------------------------------------
//  Other General Cost Function
//  x0 determines the final state, swing towards whichever direction

//  Dynamics from DART
//	mDDP = std::unique_ptr<DDP>(new DDP(2000, std::bind(DartStepDynamics, std::placeholders::_1, std::placeholders::_2, mDDPWorld), std::bind(CartPoleStepCostCos, std::placeholders::_1, std::placeholders::_2, Q, R), std::bind(CartPoleFinalCostCos,std::placeholders::_1, Qf), LQR, std::make_tuple(x0,xd,1)));


}
