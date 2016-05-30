#ifndef CARTPOLEUTILITY_H
#define CARTPOLEUTILITY_H

#include "dart/dart.h"

namespace CartPoleUtility{

using namespace Eigen;

VectorXd CartPoleStepDynamics(VectorXd xi, VectorXd ui, double m_c = 1, double m_p =1, double l = 0.5, double g = 9.81, double delta_t = 0.001);

}

#endif
