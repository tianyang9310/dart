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
    x_fly       = Eigen::MatrixXd::Zero(mController->mDDP->x_dim, mController->mDDP->T);

    std::cout<<"########################################"<<std::endl;
    std::cout<<" Begin Differential Dynamic Programming "<<std::endl;
    std::cout<<"########################################"<<std::endl;
}

inline void MyWindow::mPointer_Debug()
{
#ifdef mPOINTER_DEBUG
    std::cout<<&mWorld<<std::endl;
    std::cout<<mWorld<<std::endl;
    std::cout<<mSnapshot<<std::endl;
    std::cout<<"mWorld pointer reference count is "<<mWorld.use_count()<<std::endl;
#endif
}

inline void MyWindow::mDofStat()
{
#ifdef mDOFSTAT
    for (size_t i = 0; i<mWorld->getSkeleton("mDoubleCartPole")->getNumDofs();i++)
    {
        std::cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof(i)->getName()<<"  position: "<<mWorld->getSkeleton("mDoubleCartPole")->getDof(i)->getPosition()<<"        velocity: "<<mWorld->getSkeleton("mDoubleCartPole")->getDof(i)->getVelocity()<<"        acceleration: "<<mWorld->getSkeleton("mDoubleCartPole")->getDof(i)->getAcceleration()<<std::endl;
    }
#endif
}


void MyWindow::timeStepping() 
{
    std::cout<<'\r'<<mWorld->getTime();
    if (mWorld->getSimFrames()==mController->mDDP->T-1)
    {
        cout<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->getPosition()<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->getPosition()<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_pole_pole2")->getPosition()<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->getVelocity()<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->getVelocity()<<endl;
        cout<<mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_pole_pole2")->getVelocity()<<endl;

        // reset x to zero
        // When clone the world, x0 is automatically reset to 0
        for (int __=0; __<100; __++)
        mController->mDDP->trajopt();


        std::cout<<"########################################"<<std::endl;
        std::cout<<"       Finish "<<++mDDP_iter<<"th DDP iteration        "<<std::endl;
        std::cout<<"########################################"<<std::endl;
        //mPointer_Debug();
        //mDofStat();
        setWorld(mSnapshot->clone());
        //mDofStat();
        //mPointer_Debug();

        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->setPosition((mController->x0)(0));
        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->setPosition((mController->x0)(1));
        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_pole_pole2")->setPosition((mController->x0)(2));
        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->setVelocity((mController->x0)(3));
        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->setVelocity((mController->x0)(4));
        mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_pole_pole2")->setVelocity((mController->x0)(5));

        // pause for user interactive control
    //  std::cin.get();
    }
    // apply external force via u[i]
    int mSimFrameCount;
    mSimFrameCount = mWorld->getSimFrames();
#ifdef mDYNAMICS_DEBUG
    std::cout<<mSimFrameCount<<"th control is "<<mController->mDDP->u.col(mSimFrameCount)<<std::endl;
#endif
    mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->setForce(mController->mDDP->u.col(mSimFrameCount)[0]); 
//------------------------------------------------------------------------------------------------
    SimWindow::timeStepping();
//------------------------------------------------------------------------------------------------
#ifdef mDYNAMICS_DEBUG
    // bookkeeping x[i+1]
    mSimFrameCount = mWorld->getSimFrames();
    std::cout<<mSimFrameCount<<"th step state is"<<std::endl;
    x_fly(0,mSimFrameCount) = mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->getPosition();
    x_fly(1,mSimFrameCount) = mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->getPosition();
    x_fly(2,mSimFrameCount) = mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_hold_cart")->getVelocity();
    x_fly(3,mSimFrameCount) = mWorld->getSkeleton("mDoubleCartPole")->getDof("Joint_cart_pole")->getVelocity();
    std::cout<<x_fly.col(mSimFrameCount).transpose()<<std::endl;
    std::cin.get();
#endif
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
        mWorld->getSkeleton("mDoubleCartPole")->getDof(0)->setForce(100);
        break;

        case 'd':
        mWorld->getSkeleton("mDoubleCartPole")->getDof(0)->setForce(-100);
        break;

        case 'r':
        mWorld->getSkeleton("mDoubleCartPole")->getDof(0)->setPosition(0);
        break;

    //  case 'o':
    //  std::cout<<mWorld->getSkeleton("mDoubleCartPole")->getBodyNode(0)->getTransform().matrix()<<std::endl;
    //  break;

        default:
        SimWindow::keyboard(_key, _x, _y);
        break;
    }
}
