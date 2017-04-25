#include "utils.h"

namespace CntctLrnin {

//==============================================================================
Eigen::MatrixXd getTangentBasisMatrixLemke(const Eigen::Vector3d& _n,
                                           int numBasis) {
  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  Eigen::MatrixXd T(Eigen::MatrixXd::Zero(3, numBasis));

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3d tangent = Eigen::Vector3d::UnitZ().cross(_n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close, pick another tangent (use X-axis as arbitrary vector)
  if (tangent.norm() < DART_CONTACT_CONSTRAINT_EPSILON)
    tangent = Eigen::Vector3d::UnitX().cross(_n);

  tangent.normalize();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  T.col(0) = tangent;
  for (size_t mIdxBasis = 1; mIdxBasis < numBasis; mIdxBasis++) {
    T.col(mIdxBasis) = Eigen::Quaterniond(Eigen::AngleAxisd(
                           DART_PI_HALF / (double(numBasis) / 4.0), _n)) *
                       T.col(mIdxBasis - 1);
    if (T.col(mIdxBasis).dot(_n) > MY_DART_ZERO) {
      dterr << "Error in constructing basis matrix" << std::endl;
    }
  }

  return T;
}

//==============================================================================
std::string idx2string(const int idxmBox) {
  std::string name;
  name = "BodyNode_" + std::to_string(idxmBox);
  return name;
}

//==============================================================================
simulation::WorldPtr myReadWorld(
    tinyxml2::XMLElement* _worldElement, const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever) {
  assert(_worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld(new MyWorld());

  // set DART collision detector
  newWorld->getConstraintSolver()->setCollisionDetector(
      // new dart::collision::BulletCollisionDetector());
      // new dart::collision::FCLMeshCollisionDetector());
      new dart::collision::DARTCollisionDetector());

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement =
      _worldElement->FirstChildElement("physics");
  if (physicsElement != nullptr) {
    // Time step
    tinyxml2::XMLElement* timeStepElement = nullptr;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != nullptr) {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = nullptr;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != nullptr) {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector")) {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh") {
        newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::FCLMeshCollisionDetector());
      } else if (strCD == "fcl") {
        newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::FCLCollisionDetector());
      } else if (strCD == "dart") {
        newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::DARTCollisionDetector());
      }
#ifdef HAVE_BULLET_COLLISION
      else if (strCD == "bullet") {
        newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::BulletCollisionDetector());
      }
#endif
      else {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    } else {
      newWorld->getConstraintSolver()->setCollisionDetector(
          new collision::FCLMeshCollisionDetector());
    }
  }

  //--------------------------------------------------------------------------
  // Load soft skeletons
  ElementEnumerator SkeletonElements(_worldElement, "skeleton");
  while (SkeletonElements.next()) {
    dynamics::SkeletonPtr newSkeleton =
        SkelParser::readSkeleton(SkeletonElements.get(), _baseUri, _retriever);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
simulation::WorldPtr myReadWorld(
    const common::Uri& _uri, const common::ResourceRetrieverPtr& _retriever) {
  const common::ResourceRetrieverPtr retriever =
      SkelParser::getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try {
    openXMLFile(_dartFile, _uri, retriever);
  } catch (std::exception const& e) {
    dterr << "[SkelParser::readWorld] LoadFile [" << _uri.toString()
          << "] Failed: " << e.what() << "\n";
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartFile);
  if (!worldElement) {
    dterr << "[SkelParser::readWorld] File named [" << _uri.toString()
          << "] could not be parsed!\n";
    return nullptr;
  }

  return myReadWorld(worldElement, _uri, retriever);
}

//==============================================================================
tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document) {
  //--------------------------------------------------------------------------
  // Check xml tag
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = _document.FirstChildElement("skel");
  if (skelElement == nullptr) {
    dterr << "XML Document does not contain <skel> as the root element.\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == nullptr) {
    dterr << "XML Document does not contain a <world> element under the <skel> "
          << "element.\n";
    return nullptr;
  }

  return worldElement;
}
}  // namespace CntctLrnin
