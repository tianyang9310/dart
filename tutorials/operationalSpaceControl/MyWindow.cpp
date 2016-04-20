/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "MyWindow.h"

#include <iostream>
#include <fstream>

using namespace dart::common;
using namespace dart::utils;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

//==============================================================================
MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),
    mController(_controller),
    mCircleTask(false)
{
  assert(_controller != nullptr);

  // Set the initial target positon to the initial position of the end effector
  mTargetPosition = mController->getEndEffector()->getTransform().translation();
  mShowMarkers = false;
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  if (mCircleTask)
  {
    static double time = 0.0;
    const double dt = 0.0005;
    const double radius = 0.2;
	double height = mTargetPosition[2];
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.0, 1.0);

    mTargetPosition = center;
	mTargetPosition[2] = height;
    mTargetPosition[0] = radius * std::cos(time);
    mTargetPosition[1] = radius * std::sin(time);
    //mTargetPosition[2] = radius * std::cos(time);

    time += dt;
  }

  // Update the controller and apply control force to the robot
  mController->update(mTargetPosition);
  
  // keep positions, velocities, accelerations and forces into a file
  print2File(mWorld->getSkeleton("manipulator"), true);

  // Step forward the simulation
  mWorld->step();

  print2File(mWorld->getSkeleton("manipulator"), false);


}

void MyWindow::print2File(dart::dynamics::SkeletonPtr mManipulator, bool mDynaics)
{
	std::ofstream myFile;
	myFile.open("training_data.dat", std::ofstream::out|std::ofstream::app);
	if (myFile.is_open())
	{
		if (mDynaics)
		{
			myFile<<mManipulator->getForces().transpose();
		}
		else
		{
			myFile<<mManipulator->getPositions().transpose()<<"		"<<mManipulator->getVelocities().transpose()<<"		"<<mManipulator->getAccelerations().transpose()<<std::endl;
		}
		myFile.close();
	}
	else 
	{
		std::cout<<"cannot open the data file"<<std::endl;
	}

}

//==============================================================================
void MyWindow::drawSkels()
{
  // Draw the target position
//  if (mRI)
//  {
//    mRI->setPenColor(Eigen::Vector4d(0.8, 0.2, 0.2,0.0));
//    mRI->pushMatrix();
//    mRI->translate(mTargetPosition);
//    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
//    mRI->popMatrix();
//  }

  //glEnable(GL_LIGHTING);
  //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // Draw skeletons
  SimWindow::drawSkels();
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  double incremental = 0.01;

  switch (_key)
  {
    case 'c':  // print debug information
      if (mCircleTask)
      {
        std::cout << "Circle task [off]." << std::endl;
        mCircleTask = false;
      }
      else
      {
        std::cout << "Circle task [on]." << std::endl;
        mCircleTask = true;
      }
      break;
    case 'q':
      mTargetPosition[0] -= incremental;
      break;
    case 'w':
      mTargetPosition[0] += incremental;
      break;
    case 'a':
      mTargetPosition[1] -= incremental;
      break;
    case 's':
      mTargetPosition[1] += incremental;
      break;
    case 'z':
      mTargetPosition[2] -= incremental;
      break;
    case 'x':
      mTargetPosition[2] += incremental;
      break;
    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}

