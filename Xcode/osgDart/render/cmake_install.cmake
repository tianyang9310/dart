# Install script for directory: /Users/Yang/Material/Research/dart/osgDart/render

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgDart/render" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/osgDart/render/BoxShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/CylinderShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/EllipsoidShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/LineSegmentShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/MeshShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/PlaneShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/ShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/SoftMeshShapeNode.h"
    "/Users/Yang/Material/Research/dart/osgDart/render/WarningShapeNode.h"
    "/Users/Yang/Material/Research/dart/Xcode/osgDart/render/render.h"
    )
endif()

