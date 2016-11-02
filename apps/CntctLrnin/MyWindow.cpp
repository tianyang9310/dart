#include "MyWindow.h"

using namespace std;

MyWindow::MyWindow(dart::simulation::WorldPtr world)
{
    setWorld(world);
    mCollisionDetector = std::unique_ptr<dart::collision::CollisionDetector>(mWorld->getConstraintSolver()->getCollisionDetector());
    counter = 0;
}

MyWindow::~MyWindow()
{
    
}

void MyWindow::timeStepping()
{
    // lock Skeleton Velocities before stepping function
    dynamic_cast<MyDantzigLCPSolver*>(mWorld->getConstraintSolver()->getLCPSolver())->pushVelocities(
            mWorld->getSkeleton("mBox"),
            mWorld->getSkeleton("mBox")->getVelocities());
    dynamic_cast<MyDantzigLCPSolver*>(mWorld->getConstraintSolver()->getLCPSolver())->pushVelocities(
            mWorld->getSkeleton("ground skeleton"),
            mWorld->getSkeleton("ground skeleton")->getVelocities());

    dart::gui::SimWindow::timeStepping();

    dynamic_cast<MyDantzigLCPSolver*>(mWorld->getConstraintSolver()->getLCPSolver())->getSkeletonVelocitiesLock().clear();
    // std::cout<<"Clear velocities lock "<<dynamic_cast<MyDantzigLCPSolver*>(mWorld->getConstraintSolver()->getLCPSolver())->getSkeletonVelocitiesLock().size()<<std::endl;

    // regularization the positive of cube
    double range = 12.5;
    if ((std::abs(mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->getTransform().translation()(0)) > range)       // x direction
        || (std::abs(mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->getTransform().translation()(2)) > range))  // z direction
    {
        dterr<<"ERROR: runnng out of range, need regularization!!!"<<std::endl;
        Eigen::Vector3d init_pos(0.0, 1.0, 0.0);
        Eigen::Quaterniond init_ori_Quat;  // arbitrary initial orientation
        init_ori_Quat.w() = 1.0;
        init_ori_Quat.vec() = Eigen::Vector3d::Random();
        init_ori_Quat.normalize();
        Eigen::Matrix3d init_ori = init_ori_Quat.toRotationMatrix();

        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.translation() = init_pos;
        tf.linear() = init_ori;
        mWorld->getSkeleton("mBox")->setPositions(Eigen::VectorXd::Zero(6));
        mWorld->getSkeleton("mBox")->setVelocities(Eigen::VectorXd::Zero(6));
    }

    counter = (counter + 1) % 200;

    if (counter == 50)
    {
        addExtForce();
    }
    else if (counter == 150)
    {
        addExtTorque();
    }
}

void MyWindow::addExtForce()
{
    // mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtTorque(Eigen::Vector3d::Random()*100);
    
    Eigen::Vector3d extForce = Eigen::Vector3d::Random() * 5e2;
    extForce[1] = 0.0;
    mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(extForce);
    // mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtForce(Eigen::Vector3d::Random()*3);
}

void MyWindow::addExtTorque()
{
    mWorld->getSkeleton("mBox")->getBodyNode("BodyNode_1")->addExtTorque(Eigen::Vector3d::Random()*100);
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
    switch (_key)
    {
        case 'a':
            addExtForce();
            break;
        case 'b':
            addExtTorque();
            break;
        default:
            dart::gui::SimWindow::keyboard(_key,_x,_y);
    }
    
}

void MyWindow::drawSkels()
{
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    dart::gui::SimWindow::drawSkels();
}
