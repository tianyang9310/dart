# Install script for directory: /Users/Yang/Material/Research/dart/dart/constraint

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/constraint" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/constraint/BalanceConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/BallJointConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/ConstrainedGroup.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/ConstraintBase.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/ConstraintSolver.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/ContactConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/DantzigLCPSolver.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/JointConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/JointCoulombFrictionConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/JointLimitConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/LCPSolver.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/PGSLCPSolver.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/SoftContactConstraint.h"
    "/Users/Yang/Material/Research/dart/dart/constraint/WeldJointConstraint.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/constraint/constraint.h"
    )
endif()

