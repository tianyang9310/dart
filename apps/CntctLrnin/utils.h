#ifndef MYUTILS
#define MYUTILS

#include <tinyxml2.h>
#include <string>
#include "MyWorld.h"
#include "dart/dart.h"

#ifndef DART_CONTACT_CONSTRAINT_EPSILON
#define DART_CONTACT_CONSTRAINT_EPSILON 1e-6
#endif

#ifndef MY_DART_ZERO
#define MY_DART_ZERO 1e-6
#endif

#ifndef CLAMP_ZERO
#define CLAMP_ZERO 1e-8
#endif

// #define REGULARIZED_PRINT // print regularized info

namespace dart {

namespace dynamics {
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class PlanarJoint;
class FreeJoint;
class Marker;
}

namespace dynamics {
class BodyNode;
class Shape;
class Skeleton;
}

namespace simulation {
class World;
}
}

namespace CntctLrnin {

using namespace dart;
using namespace dart::utils;

Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis);

template <typename T>
void clampZero(T& z) {
  // manually regularize Lemke solution right before apply impulse
  // after regularization, w might not satisfy LCP condition
  //
  // Zero regularization
  auto oldZ = z;
  auto myZero = z;
  myZero.setZero();
  z = (z.array() > -CLAMP_ZERO && z.array() < CLAMP_ZERO).select(myZero, z);
  if (((oldZ).array() > -CLAMP_ZERO && (oldZ).array() < CLAMP_ZERO &&
       (oldZ).array() != 0)
          .any()) {
#ifdef REGULARIZED_PRINT
    dtwarn << "Before zero regularized: z is " << oldZ.transpose() << std::endl;
    dtwarn << "After zero regularized: z is " << z.transpose() << std::endl;
#endif
  }
}

template <typename T1, typename T2>
void clampNeg(T1& z, T2 Validation) {
  // Negative regularization
  auto oldZ = z;
  auto myZero = z;
  myZero.setZero();
  if (Validation) {
    oldZ = z;
    z = (z.array() < -CLAMP_ZERO).select(myZero, z);
    if (((oldZ).array() < -CLAMP_ZERO).any()) {
#ifdef REGULARIZED_PRINT
      dtwarn << "Before negative regularized: z is " << oldZ.transpose()
             << std::endl;
      dtwarn << "After negative regularized: z is" << z.transpose()
             << std::endl;
#endif
    }
  }
}

std::string idx2string(const int idxmBox);

simulation::WorldPtr myReadWorld(
    tinyxml2::XMLElement* _worldElement, const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

simulation::WorldPtr myReadWorld(
    const common::Uri& _uri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);

tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document);

}  // namespace CntctLrnin

#endif  // MYUTILS
