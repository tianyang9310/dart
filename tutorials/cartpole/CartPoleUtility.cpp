#include "CartPoleUtility.h"

namespace CartPoleUtility{

VectorXd CartPoleStepDynamics(VectorXd xi, VectorXd ui, double m_c, double m_p, double l, double g, double delta_t)
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

}
