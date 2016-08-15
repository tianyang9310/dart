#include "MyWindow.h"
//#define mPOINTER_DEBUG
//#define mDOFSTAT
//#define mDYNAMICS_DEBUG

MyWindow::MyWindow(WorldPtr world)
{
    setWorld(world);
    mSnapshot   = mWorld->clone();
    mController = std::unique_ptr<Controller>(new Controller(mSnapshot->clone()));
    mDDP_iter   = 0;

    std::cout<<"########################################"<<std::endl;
    std::cout<<" Begin Differential Dynamic Programming "<<std::endl;
    std::cout<<"########################################"<<std::endl;
}

void MyWindow::timeStepping() 
{
    std::cout<<'\r'<<mWorld->getTime();
    if (mWorld->getSimFrames()==mController->mMPC->T-1)
    {
        cout<<endl;
        cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition()<<endl;
        cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition()<<endl;
        cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity()<<endl;
        cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity()<<endl;

        // reset x to zero
        // When clone the world, x0 is automatically reset to 0
        std::cout<<std::endl;
        mController->mMPC->run();


        std::cout<<"########################################"<<std::endl;
        std::cout<<"       Finish "<<++mDDP_iter<<"th DDP iteration        "<<std::endl;
        std::cout<<"########################################"<<std::endl;
        //mPointer_Debug();
        //mDofStat();
        setWorld(mSnapshot->clone());
        //mDofStat();
        //mPointer_Debug();

        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition((mController->x0)(0));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition((mController->x0)(1));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity((mController->x0)(2));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity((mController->x0)(3));

        // pause for user interactive control
    //  std::cin.get();
    }
    // apply external force via u[i]
    int mSimFrameCount;
    mSimFrameCount = mWorld->getSimFrames();
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mController->mMPC->u.col(mSimFrameCount)[0]); 
//------------------------------------------------------------------------------------------------
    SimWindow::timeStepping();
//------------------------------------------------------------------------------------------------
}

void MyWindow::drawSkels() 
{
//  glEnable(GL_LIGHTING);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
    switch(_key)
    {
        case 'a':
        mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(100);
        break;

        case 'd':
        mWorld->getSkeleton("mCartPole")->getDof(0)->setForce(-100);
        break;

        case 'r':
        mWorld->getSkeleton("mCartPole")->getDof(0)->setPosition(0);
        break;

    //  case 'o':
    //  std::cout<<mWorld->getSkeleton("mCartPole")->getBodyNode(0)->getTransform().matrix()<<std::endl;
    //  break;

        default:
        SimWindow::keyboard(_key, _x, _y);
        break;
    }
}
