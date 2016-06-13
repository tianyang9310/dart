#ifndef CARTPOLEUTILITY_H
#define CARTPOLEUTILITY_H

#include "dart/dart.h"
#include "../../DDP/libDDP/type.h"

namespace DoubleCartPoleUtility{

using namespace Eigen;


VectorXd DartStepDynamics(VectorXd xi, VectorXd ui, dart::simulation::WorldPtr mWorld);

Scalar CartPoleStepCost(const VectorXd xi, const VectorXd ui, const VectorXd xd, const MatrixXd Q, const MatrixXd R);

Scalar CartPoleFinalCost(const VectorXd xT, const VectorXd xd, const MatrixXd Qf);

Scalar CartPoleStepCostCos(const VectorXd xi, const VectorXd ui, const MatrixXd Q, const MatrixXd R);

Scalar CartPoleFinalCostCos(const VectorXd xT, const MatrixXd Qf);

}

#endif
