#ifndef CARTPOLEUTILITY_H
#define CARTPOLEUTILITY_H

#include "dart/dart.h"
#include "type.h"

namespace CartPoleUtility{

using namespace Eigen;

VectorXd CartPoleStepDynamics(const VectorXd xi, const VectorXd ui, double m_c = 1, double m_p =1, double l = 0.5, double g = 9.81, double delta_t = 0.001);

VectorXd DartStepDynamics(VectorXd xi, VectorXd ui, dart::simulation::WorldPtr mWorld);

Scalar CartPoleStepCost(const VectorXd xi, const VectorXd ui, const VectorXd xd, const MatrixXd Q, const MatrixXd R);

Scalar CartPoleFinalCost(const VectorXd xT, const VectorXd xd, const MatrixXd Qf);

}

#endif
