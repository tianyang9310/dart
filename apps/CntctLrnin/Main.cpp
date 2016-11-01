#include <iostream>
#include <ctime>
#include <chrono>

#include "dart/dart.h"
#include "MyWindow.h"
#include "addSkeles.h"
#include "MyDantzigLCPSolver.h"


int main(int argc, char* argv[])
{
    std::srand((unsigned) (std::chrono::system_clock::now().time_since_epoch().count()));
    // create and initialize the world
    dart::simulation::WorldPtr mWorld = std::make_shared<dart::simulation::World>();
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
    std::cout<<"Ground Skeleton: "<<mWorld->getSkeleton("ground skeleton")->isMobile()<<std::endl;
    std::cout<<"mBox: "<<mWorld->getSkeleton("mBox")->isMobile()<<std::endl;;
    mWorld->getConstraintSolver()->setLCPSolver(new MyDantzigLCPSolver(mWorld->getTimeStep(), totalDOF));
    
    // create a window and link it to the world
    MyWindow window(mWorld);
    
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();
    
    return 0;
}

