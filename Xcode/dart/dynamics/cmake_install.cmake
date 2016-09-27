# Install script for directory: /Users/Yang/Material/Research/dart/dart/dynamics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "headers")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/dynamics" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/dynamics/ArrowShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/AssimpInputResourceAdaptor.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/BallJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/BodyNode.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/BoxShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Branch.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Chain.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/CylinderShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/DegreeOfFreedom.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/EllipsoidShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/EndEffector.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Entity.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/EulerJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/FixedFrame.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Frame.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/FreeJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Group.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/HierarchicalIK.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Inertia.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/InvalidIndex.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/InverseKinematics.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/JacobianNode.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Joint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/LineSegmentShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Linkage.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Marker.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/MeshShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/MetaSkeleton.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/MultiDofJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Node.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/PlanarJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/PlaneShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/PointMass.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/PrismaticJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/ReferentialSkeleton.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/RevoluteJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/ScrewJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Shape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/SimpleFrame.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/SingleDofJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/Skeleton.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/SmartPointer.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/SoftBodyNode.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/SoftMeshShape.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/TemplatedJacobianNode.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/TranslationalJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/UniversalJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/WeldJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/ZeroDofJoint.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/dynamics/dynamics.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "headers")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/dynamics/detail" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/BodyNode.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/BodyNodePtr.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/DegreeOfFreedomPtr.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/InverseKinematics.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/InverseKinematicsPtr.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/JointPtr.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/MultiDofJoint.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/NodePtr.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/Skeleton.h"
    "/Users/Yang/Material/Research/dart/dart/dynamics/detail/TemplatedJacobianNode.h"
    )
endif()

