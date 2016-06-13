#include "DoubleCartPoleUtility.h"

namespace DoubleCartPoleUtility{

VectorXd DartStepDynamics(VectorXd xi, VectorXd ui, dart::simulation::WorldPtr mWorld)
{
// --------------------------------------------------
//	Dart Step Dynamics Function
//	Computing xi_1 according to xi and ui using dart simulation
//	Note:
//		Directly use the World Pointer passed in.
// --------------------------------------------------
	dart::dynamics::SkeletonPtr mDoubleCartPole = mWorld->getSkeleton("mDoubleCartPole");
	VectorXd xi_1(xi.rows());

	mDoubleCartPole->getDof("Joint_hold_cart")->setPosition(xi(0));
	mDoubleCartPole->getDof("Joint_cart_pole")->setPosition(xi(1));
	mDoubleCartPole->getDof("Joint_pole_pole2")->setPosition(xi(2));
	mDoubleCartPole->getDof("Joint_hold_cart")->setVelocity(xi(3));
	mDoubleCartPole->getDof("Joint_cart_pole")->setVelocity(xi(4));
	mDoubleCartPole->getDof("Joint_pole_pole2")->setVelocity(xi(5));

	mDoubleCartPole->getDof("Joint_hold_cart")->setForce(ui(0));	

	mWorld->step();

	xi_1(0) = mDoubleCartPole->getDof("Joint_hold_cart")->getPosition();
	xi_1(1) = mDoubleCartPole->getDof("Joint_cart_pole")->getPosition();
	xi_1(2) = mDoubleCartPole->getDof("Joint_pole_pole2")->getPosition();
	xi_1(3) = mDoubleCartPole->getDof("Joint_hold_cart")->getVelocity();
	xi_1(4) = mDoubleCartPole->getDof("Joint_cart_pole")->getVelocity();
	xi_1(5) = mDoubleCartPole->getDof("Joint_pole_pole2")->getVelocity();

	return xi_1;
}

Scalar CartPoleStepCost(const VectorXd xi, const VectorXd ui, const VectorXd xd, const MatrixXd Q, const MatrixXd R)
{
// --------------------------------------------------
//	CartPole Step Cost Function
//	Computing ci according to xi and ui 
// --------------------------------------------------
	return (0.5*(xi-xd).transpose()*Q*(xi-xd) + 0.5*ui.transpose()*R*ui);
}

Scalar CartPoleFinalCost(const VectorXd xT, const VectorXd xd, const MatrixXd Qf)
{
// --------------------------------------------------
//	CartPole Final Cost Function
//	Computing cT according to xT
// --------------------------------------------------
	return ( 0.5*(xT-xd).transpose()*Qf*(xT-xd) );
}


Scalar CartPoleStepCostCos(const VectorXd xi, const VectorXd ui, const MatrixXd Q, const MatrixXd R)
{
// --------------------------------------------------
//	CartPole Step Cost Function
//	Computing ci according to xi and ui 
// --------------------------------------------------
	VectorXd xi_aug = xi;
	xi_aug(1) = 5 + cos(xi(1));
	return (0.5*xi.transpose()*Q*xi + 0.5*ui.transpose()*R*ui);
}

Scalar CartPoleFinalCostCos(const VectorXd xT, const MatrixXd Qf)
{
// --------------------------------------------------
//	CartPole Final Cost Function
//	Computing cT according to xT
// --------------------------------------------------
	VectorXd xT_aug = xT;
	xT_aug(1) = 5 + cos(xT(1));
	return ( 0.5*xT_aug.transpose()*Qf*xT_aug );
}

}
