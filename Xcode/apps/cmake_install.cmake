# Install script for directory: /Users/Yang/Material/Research/dart/apps

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/Yang/Material/Research/dart/Xcode/apps/addDeleteSkels/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/atlasSimbicon/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/bipedStand/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/hardcodedDesign/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/hybridDynamics/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/jointConstraints/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/mixedChain/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/operationalSpaceControl/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/rigidBodies/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/rigidChain/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/rigidCubes/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/rigidLoop/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/rigidShapes/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/simpleFrames/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/softBodies/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/speedTest/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/apps/vehicle/cmake_install.cmake")

endif()

