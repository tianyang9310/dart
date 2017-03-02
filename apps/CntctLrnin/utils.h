#ifndef CNTCTLRNIN_UTILS_H
#define CNTCTLRNIN_UTILS_H

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

/// Get friction cone basis matrix
Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis);

/// Get string from index
std::string idx2string(const int idxmBox);

/// Override built-in read world function for separate use
simulation::WorldPtr myReadWorld(
    tinyxml2::XMLElement* _worldElement, const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

simulation::WorldPtr myReadWorld(
    const common::Uri& _uri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);

tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document);

}  // namespace CntctLrnin

#endif  // CNTCTLRNIN_UTILS

