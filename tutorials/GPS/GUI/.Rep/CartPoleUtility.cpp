#include "CartPoleUtility.h"

namespace CartPoleUtility{

VectorXd CartPoleStepDynamics(const VectorXd xi, const VectorXd ui, double m_c, double m_p, double l, double g, double delta_t)
{
// --------------------------------------------------
//	CartPole Step Dynamics Function
//	Computing xi_1 according to xi and ui using equations of motion
//	Note:
//		The direction of \theta in DART and equations of motion are different,
//		therefore the equations are changed accordingly
// --------------------------------------------------
	VectorXd xi_1(xi.rows());

	double SXi_1	  = sin(xi(1));
	double CXi_1 	  = cos(xi(1));
	double Xi_3_2	  = pow(xi(3),2);
	double denomiator = m_c+m_p*pow(SXi_1,2);
	
	xi_1(2) = xi(2) + delta_t * ( - m_p*SXi_1*( l*Xi_3_2 + g*CXi_1 ))/denomiator + delta_t * ui(0)/denomiator;

	xi_1(3) = xi(3) + delta_t * ( - m_p*l*Xi_3_2*SXi_1*CXi_1 - (m_c+m_p)*g*SXi_1)/(l*denomiator) + delta_t * (  CXi_1 * ui(0))/(l*denomiator);

	xi_1(0) = xi(0) + delta_t * xi_1(2);

	xi_1(1) = xi(1) + delta_t * xi_1(3);
	
	return xi_1;
}

VectorXd DartStepDynamics(VectorXd xi, VectorXd ui, dart::simulation::WorldPtr mWorld)
{
// --------------------------------------------------
//	Dart Step Dynamics Function
//	Computing xi_1 according to xi and ui using dart simulation
//	Note:
//		Directly use the World Pointer passed in.
// --------------------------------------------------
	dart::dynamics::SkeletonPtr mCartPole = mWorld->getSkeleton("mCartPole");
	VectorXd xi_1(xi.rows());

	mCartPole->getDof("Joint_hold_cart")->setPosition(xi(0));
	mCartPole->getDof("Joint_cart_pole")->setPosition(xi(1));
	mCartPole->getDof("Joint_hold_cart")->setVelocity(xi(2));
	mCartPole->getDof("Joint_cart_pole")->setVelocity(xi(3));

	mCartPole->getDof("Joint_hold_cart")->setForce(ui(0));	

	mWorld->step();

	xi_1(0) = mCartPole->getDof("Joint_hold_cart")->getPosition();
	xi_1(1) = mCartPole->getDof("Joint_cart_pole")->getPosition();
	xi_1(2) = mCartPole->getDof("Joint_hold_cart")->getVelocity();
	xi_1(3) = mCartPole->getDof("Joint_cart_pole")->getVelocity();

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
