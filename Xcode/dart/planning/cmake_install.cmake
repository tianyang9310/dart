# Install script for directory: /Users/Yang/Material/Research/dart/dart/planning

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/planning" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/planning/Path.h"
    "/Users/Yang/Material/Research/dart/dart/planning/PathFollowingTrajectory.h"
    "/Users/Yang/Material/Research/dart/dart/planning/PathPlanner.h"
    "/Users/Yang/Material/Research/dart/dart/planning/PathShortener.h"
    "/Users/Yang/Material/Research/dart/dart/planning/RRT.h"
    "/Users/Yang/Material/Research/dart/dart/planning/Trajectory.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/planning/planning.h"
    )
endif()

