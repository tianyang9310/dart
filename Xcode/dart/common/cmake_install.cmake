# Install script for directory: /Users/Yang/Material/Research/dart/dart/common

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/common" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/common/Console.h"
    "/Users/Yang/Material/Research/dart/dart/common/Deprecated.h"
    "/Users/Yang/Material/Research/dart/dart/common/LocalResource.h"
    "/Users/Yang/Material/Research/dart/dart/common/LocalResourceRetriever.h"
    "/Users/Yang/Material/Research/dart/dart/common/Memory.h"
    "/Users/Yang/Material/Research/dart/dart/common/NameManager.h"
    "/Users/Yang/Material/Research/dart/dart/common/Observer.h"
    "/Users/Yang/Material/Research/dart/dart/common/Resource.h"
    "/Users/Yang/Material/Research/dart/dart/common/ResourceRetriever.h"
    "/Users/Yang/Material/Research/dart/dart/common/Signal.h"
    "/Users/Yang/Material/Research/dart/dart/common/sub_ptr.h"
    "/Users/Yang/Material/Research/dart/dart/common/Subject.h"
    "/Users/Yang/Material/Research/dart/dart/common/Timer.h"
    "/Users/Yang/Material/Research/dart/dart/common/Uri.h"
    "/Users/Yang/Material/Research/dart/Xcode/dart/common/common.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "headers")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dart/common/detail" TYPE FILE FILES
    "/Users/Yang/Material/Research/dart/dart/common/detail/ConnectionBody.h"
    "/Users/Yang/Material/Research/dart/dart/common/detail/NameManager.h"
    "/Users/Yang/Material/Research/dart/dart/common/detail/Signal.h"
    "/Users/Yang/Material/Research/dart/dart/common/detail/sub_ptr.h"
    )
endif()

