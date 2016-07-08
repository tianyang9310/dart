#include "MyWindow.h"

MyWindow::MyWindow(WorldPtr world, unique_ptr<GPS> _mGPS):mGPS(std::move(_mGPS))
{
    setWorld(world);
    mSnapShot = mWorld->clone();

    mGPS->run();
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->x0Bundle[2](0));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->x0Bundle[2](1));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->x0Bundle[2](2));
    mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->x0Bundle[2](3));
}

void MyWindow::timeStepping() 
{
    if (mWorld->getSimFrames() == mGPS->T-1)
    {
        mGPS->run();
        setWorld(mSnapShot->clone());
        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setPosition(mGPS->x0Bundle[2](0));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setPosition(mGPS->x0Bundle[2](1));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setVelocity(mGPS->x0Bundle[2](2));
        mWorld->getSkeleton("mCartPole")->getDof("Joint_cart_pole")->setVelocity(mGPS->x0Bundle[2](3));
    }
    // int mSimFrameCount = mWorld->getSimFrames();
    // mWorld->getSkeleton("mCartPole")->getDof("Joint_hold_cart")->setForce(mGPS->mDDP->u.col(mSimFrameCount)[0]);

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
