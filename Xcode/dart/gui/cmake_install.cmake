# Install script for directory: /Users/Yang/Material/Research/dart/dart/gui

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/gui" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/gui/GLFuncs.h"
    "/Users/Yang/Material/Research/dart/dart/gui/GlutWindow.h"
    "/Users/Yang/Material/Research/dart/dart/gui/GraphWindow.h"
    "/Users/Yang/Material/Research/dart/dart/gui/Jitter.h"
    "/Users/Yang/Material/Research/dart/dart/gui/LoadGlut.h"
    "/Users/Yang/Material/Research/dart/dart/gui/lodepng.h"
    "/Users/Yang/Material/Research/dart/dart/gui/SimWindow.h"
    "/Users/Yang/Material/Research/dart/dart/gui/SoftSimWindow.h"
    "/Users/Yang/Material/Research/dart/dart/gui/Trackball.h"
    "/Users/Yang/Material/Research/dart/dart/gui/Win2D.h"
    "/Users/Yang/Material/Research/dart/dart/gui/Win3D.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/gui/gui.h"
    )
endif()

