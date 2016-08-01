#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world, unique_ptr<GPS> _mGPS):mGPS(std::move(_mGPS))
{
    setWorld(world);
    mSnapShot = mWorld->clone();

    MaxJointPos = -1e2;
    mGPS->rund();
    idxDDP = 0;
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->x0Bundle[idxDDP](0));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->x0Bundle[idxDDP](1));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->x0Bundle[idxDDP](2));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->x0Bundle[idxDDP](3));
}

void MyWindow::timeStepping() 
{
    bool DDPDisplay = false;
    if(DDPDisplay)
    {
        if (mWorld->getSimFrames() == mGPS->T-1)
        {
            cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition()<<endl;
            cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition()<<endl;
            cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity()<<endl;
            cout<<mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity()<<endl;

            setWorld(mSnapShot->clone());
            mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->x0Bundle[idxDDP](0));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->x0Bundle[idxDDP](1));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->x0Bundle[idxDDP](2));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->x0Bundle[idxDDP](3));
        }
        int mSimFrameCount = mWorld->getSimFrames();

        // mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->cur_GPSSampleLists[30]->x.col(mSimFrameCount)(0));
        // mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->cur_GPSSampleLists[30]->x.col(mSimFrameCount)(1));
        // mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->cur_GPSSampleLists[30]->x.col(mSimFrameCount)(2));
        // mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->cur_GPSSampleLists[30]->x.col(mSimFrameCount)(3));
        
        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mGPS->DDPBundle[idxDDP]->u.col(mSimFrameCount)[0]);
    }
    else
    {
        double CurJointPos = mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition();
        MaxJointPos = MaxJointPos > CurJointPos ? MaxJointPos:CurJointPos;
        if (mWorld->getSimFrames() == mGPS->T-1)
        {
            dtmsg<<"[Max Position] "<<MaxJointPos<<endl;
            dtmsg<<"[Final Angle] "<<CurJointPos<<endl;
            keyboard(' ', 0,0);
            MaxJointPos = -1e2;
            mGPS->innerloop();
            setWorld(mSnapShot->clone());
            mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->x0Bundle[idxDDP](0));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->x0Bundle[idxDDP](1));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->x0Bundle[idxDDP](2));
            mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->x0Bundle[idxDDP](3));
        }

        if (!Py_IsInitialized())  
        {
            cout<<"Python Interpreter not Initialized!!!"<<endl;
        }
        PyObject* pArgs = PyTuple_New(4);
        PyTuple_SetItem(pArgs,0, PyFloat_FromDouble(mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getPosition()));
        PyTuple_SetItem(pArgs,1, PyFloat_FromDouble(mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getPosition()));
        PyTuple_SetItem(pArgs,2, PyFloat_FromDouble(mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->getVelocity()));
        PyTuple_SetItem(pArgs,3, PyFloat_FromDouble(mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->getVelocity()));
        PyObject* pResult =  PyObject_CallMethodObjArgs(mGPS->pInstanceCaffePolicy,PyString_FromString("act"), pArgs, NULL);
        if (! pResult)
        {
            cout<<"Failing to CALL act method of Caffe Policy"<<endl;
        }
        double u_Policy;
        if (! PyArg_ParseTuple(pResult, "d", &u_Policy))
        {
            cout<<"Failing to PARSE data from act method"<<endl;
        }

        Py_DECREF(pArgs);
        Py_DECREF(pResult);

        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(u_Policy);
    }

    SimWindow::timeStepping();
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
        case 'x':
        
        break;

        default:
        SimWindow::keyboard(_key, _x, _y);
        break;
    }
}
