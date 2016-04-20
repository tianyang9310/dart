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

#include "dart/dart.h"

#include "MyWindow.h"


using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;
SkeletonPtr createFloor()
{
	
	SkeletonPtr floor = Skeleton::create("floor");
	
	// Give the floor a body
	BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>().second;

	// Give the body a shape
	std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
			Eigen::Vector3d(5, 5, 0.2));
	box->setColor(dart::Color::Fuschia(0.3));

	body->addVisualizationShape(box);
	body->addCollisionShape(box);

	// put the body into position
	Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
	tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.2 / 2.0);
	body->getParentJoint()->setTransformFromParentBodyNode(tf);

	return floor;
}

SkeletonPtr createManipulator()
{
	dart::utils::DartLoader loader;
	SkeletonPtr manipulator = loader.parseSkeleton(DART_DATA_PATH"/my_urdf/wam7.urdf");
	manipulator->setName("manipulator");
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);
	tf.translation() = Eigen::Vector3d(0, 0, 0);
	manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

	// set initial position for each DOF
	manipulator->getDof(1)->setPosition(75.0 * M_PI / 180.0);
	manipulator->getDof(3)->setPosition(-90.0 * M_PI / 180.0);
	manipulator->getDof(5)->setPosition(0.0 * M_PI / 180.0);
	

	//set Joint position limit to make the manipulator not collapse into itself.
	for (size_t i = 0 ; i < manipulator->getNumJoints(); ++ i)
	{
		manipulator->getJoint(i)->setPositionLimitEnforced(true);
	}
	
	manipulator->enableSelfCollision();
	std::cout << "Whether check self Collision:" << manipulator->isEnabledSelfCollisionCheck() << std::endl;
	std::cout << "Whether check adjacent Collision:" << manipulator->isEnabledAdjacentBodyCheck() << std::endl;

	// output bodynodes
	std::cout<<"The manipulator has the following bodynodes: "<<std::endl;
	for (size_t i = 0 ; i< manipulator->getNumBodyNodes(); i++)
	{
		std::cout<<manipulator->getBodyNode(i)->getName()<<std::endl;
	}

	// output Dof
	std::cout<<"The manipulator has the following DOF: "<<std::endl;
	for (size_t i = 0 ; i< manipulator->getNumDofs(); i++)
	{
		std::cout<<manipulator->getDof(i)->getName()<<std::endl;
	}

	

	return manipulator;
}

int main(int argc, char* argv[])
{
  // create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);

  // load skeletons
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr floor = createFloor();
  dart::dynamics::SkeletonPtr manipulator = createManipulator();
  world->addSkeleton(floor);
  world->addSkeleton(manipulator);


  // create a window and link it to the world
  MyWindow window(new Controller(manipulator, manipulator->getBodyNode("wam/wrist_palm_stump_link")));
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(1024,768, "Forward Simulation");
  glutMainLoop();

  return 0;
}
