#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world)
{
    setWorld(world);
    mSnapshot   = mWorld->clone();
    mController = std::unique_ptr<Controller>(new Controller(mSnapshot->clone()));
    mMPC_iter   = 0;

    std::cout<<"########################################"<<std::endl;
    std::cout<<" Begin Differential Dynamic Programming "<<std::endl;
    std::cout<<"########################################"<<std::endl;
}

void MyWindow::timeStepping() 
{
    std::cout<<'\r'<<mWorld->getTime();
    if (mWorld->getSimFrames()==mController->mMPC->T-1)
    // if (mWorld->getSimFrames()==mController->mMPC->local_T-1)
    {
        inner_loop();
    }
    // apply external force via u[i]
    int mSimFrameCount;
    mSimFrameCount = mWorld->getSimFrames();
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mController->mMPC->u.col(mSimFrameCount)[0]); 
//------------------------------------------------------------------------------------------------
    SimWindow::timeStepping();
//------------------------------------------------------------------------------------------------
}

void MyWindow::inner_loop()
{
    cout<<endl;
    cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition()<<endl;
    cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition()<<endl;
    cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity()<<endl;
    cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity()<<endl;


    mController->mMPC->run();
    // mController->mMPC->inner_run(mMPC_iter++);

    cout<<"After one sweep of MPC...Ctrl is "<<endl<<mController->mMPC->u<<endl;

    setWorld(mSnapshot->clone());

    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition((mController->x0)(0));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition((mController->x0)(1));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity((mController->x0)(2));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity((mController->x0)(3));
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
