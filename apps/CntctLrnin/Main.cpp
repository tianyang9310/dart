#include <iostream>

#include "dart/dart.h"
#include "MyWindow.h"
#include "addSkeles.h"
#include "MyDantzigLCPSolver.h"


int main(int argc, char* argv[])
{
    // create and initialize the world
    dart::simulation::WorldPtr mWorld = dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/ground.skel");
    assert(mWorld != nullptr);
    AddSkel(mWorld);
    
    Eigen::Vector3d gravity(0.0, -9.81, 0.0);
    mWorld->setGravity(gravity);
    
    // using Bullet Collision Detector
    mWorld->getConstraintSolver()->setCollisionDetector(new dart::collision::BulletCollisionDetector());
    
    // using MyDantzigLCPSolver
    int totalDOF = 0;
    totalDOF += mWorld->getSkeleton("ground skeleton")->getNumDofs();
    totalDOF += mWorld->getSkeleton("mBox")->getNumDofs();
    mWorld->getConstraintSolver()->setLCPSolver(new MyDantzigLCPSolver(mWorld->getTimeStep(), totalDOF));
    
    // create a window and link it to the world
    MyWindow window(mWorld);
    
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();
    
    return 0;
}

