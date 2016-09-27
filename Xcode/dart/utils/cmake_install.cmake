# Install script for directory: /Users/Yang/Material/Research/dart/dart/utils

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/utils" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/utils/C3D.h"
    "/Users/Yang/Material/Research/dart/dart/utils/CompositeResourceRetriever.h"
    "/Users/Yang/Material/Research/dart/dart/utils/FileInfoC3D.h"
    "/Users/Yang/Material/Research/dart/dart/utils/FileInfoDof.h"
    "/Users/Yang/Material/Research/dart/dart/utils/FileInfoWorld.h"
    "/Users/Yang/Material/Research/dart/dart/utils/PackageResourceRetriever.h"
    "/Users/Yang/Material/Research/dart/dart/utils/Parser.h"
    "/Users/Yang/Material/Research/dart/dart/utils/SkelParser.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/utils/utils.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/Yang/Material/Research/dart/Xcode/dart/utils/sdf/cmake_install.cmake")
  include("/Users/Yang/Material/Research/dart/Xcode/dart/utils/urdf/cmake_install.cmake")

endif()

