#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator(int _sysType) : sysType(_sysType), mTimeStep(0.0), mElapsedTime(0.0), mParticles(0), mFrameNumber(0), Integrator(3) {
    if (sysType == 0) {         //galileo experiment
        // initialize the particles
        mParticles.resize(4);

        // Init particle positions (default is 0, 0, 0)
        mParticles[0].mPosition[0] = -0.3;
        mParticles[0].mPosition[1] = 20.0;
        mParticles[1].mPosition[0] = -0.1;
        mParticles[1].mPosition[1] = 20.0;
        mParticles[2].mPosition[0] = 0.1;
        mParticles[2].mPosition[1] = 20.0;
        mParticles[3].mPosition[0] = 0.3;
        mParticles[3].mPosition[1] = 20.0;

        InitVelocity();

        // Init particle colors (default is red)
        mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
        mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
        mParticles[3].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue

        mTimeStep = 0.03;
    }
    else {  //tinker toy
        // initialize the particles
        mParticles.resize(2);

        // Init particle positions (default is 0, 0, 0)
        mParticles[0].mPosition[0] = 0.2;
        mParticles[1].mPosition[0] = 0.2;
        mParticles[1].mPosition[1] = -0.1;

        mTimeStep = 0.0003;
    } 
    ExtForce.setZero();
    flag = -1;
}


void Simulator::reset() {
    if (sysType == 0) {         //galileo experiment
        mParticles[0].mPosition[0] = -0.3;
        mParticles[0].mPosition[1] = 20.0;
        mParticles[1].mPosition[0] = -0.1;
        mParticles[1].mPosition[1] = 20.0;
        mParticles[2].mPosition[0] = 0.1;
        mParticles[2].mPosition[1] = 20.0;
        mParticles[3].mPosition[0] = 0.3;
        mParticles[3].mPosition[1] = 20.0;
    } else {            //tinker toy
        mParticles[0].mPosition[0] = 0.2;
        mParticles[0].mPosition[1] = 0.0;
        mParticles[1].mPosition[0] = 0.2;
        mParticles[1].mPosition[1] = -0.1;
    }
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    mElapsedTime = 0;    
    mFrameNumber = 0;

    view_translation.setZero();
    view_rotation.setZero();

    if (sysType == 0)
    {
        InitVelocity();
    }
    
    ExtForce.setZero();
    flag = -1;
}


//analytical solution
void Simulator::analyticSolve(Particle& p) {
    // TODO: Replace the following code with correct analytical solution using mElapsedTime
    
    // double g = -9.81;
    // p.mPosition[1] = 20.0+0.5*g*(mElapsedTime+mTimeStep)*(mElapsedTime+mTimeStep) + mInitVelocity * (mElapsedTime+mTimeStep);
    double t = mElapsedTime + mTimeStep;
    p.mPosition = (Eigen::VectorXd(3)<<-0.3, 20.0, 0.0).finished() + mInitVelocity * t + 0.5*t*t*p.mAccumulatedForce/p.mMass;

    // p.mPosition[1] -= 0.005;
}

//forward euler integrator
void Simulator::fwdEulerSolve(Particle& p) {
    // TODO: Replace the following code with fwd euler

    // double g= -9.81;
    // p.mPosition[1] = p.mPosition[1] + mTimeStep*p.mVelocity[1]; 
    // p.mVelocity[1] = p.mVelocity[1] + mTimeStep*g; 
    
    // Note: fwdEuler integrator is not accurate when compared with analytic solution
    
    ClearForce();
    ApplyForce();

    p.mPosition = p.mPosition + mTimeStep * p.mVelocity;
    p.mVelocity = p.mVelocity + mTimeStep * p.mAccumulatedForce / p.mMass;

    // p.mPosition[1] -= 0.005;
}

//midpoint integrator
void Simulator::midPointSolve(Particle& p) {
    // TODO: Replace the following code with midpoint method

    // double g= -9.81;
    // double midPos = p.mPosition[1] + 0.5*mTimeStep*p.mVelocity[1];
    // double midVel = p.mVelocity[1] + 0.5*mTimeStep*g;

    // p.mPosition[1] = p.mPosition[1] + mTimeStep*midVel;
    // p.mVelocity[1] = p.mVelocity[1] + mTimeStep*g;
    
    // Note: Midpoint integrator is accurate when compared with analytic solution

    ClearForce();
    ApplyForce();

    Eigen::Vector3d Pos0 = p.mPosition;
    Eigen::Vector3d Vel0 = p.mVelocity;

    p.mPosition = p.mPosition + 0.5*mTimeStep*p.mVelocity;
    p.mVelocity = p.mVelocity + 0.5*mTimeStep*p.mAccumulatedForce/p.mMass;

    ClearForce();
    ApplyForce();

    p.mPosition = Pos0 + mTimeStep * p.mVelocity;
    p.mVelocity = Vel0 + mTimeStep * p.mAccumulatedForce/p.mMass;

    // p.mPosition[1] -= 0.005;
}


// implicit integrator
void Simulator::ImplicitEurlerSolve(Particle& p) {
    // TODO: Replace the following code with midpoint method

    ClearForce();
    ApplyForce();

    Eigen::Vector3d Acc = p.mAccumulatedForce/p.mMass;
    Eigen::VectorXd DeltaY(6);
    Eigen::MatrixXd A(6,6);
    A.setIdentity();
    A=A/mTimeStep;
    Eigen::Matrix3d B;
    B.setIdentity();
    Eigen::Matrix3d Corner;
    Corner = (A.block<3,3>(0,3) - B).eval();
    A.block<3,3>(0,3) = Corner;
    Eigen::VectorXd b(6);
    b.block<3,1>(0,0) = p.mVelocity;
    b.block<3,1>(3,0) = Acc;
    DeltaY = (A.inverse()*b).eval();
    // DeltaY = A.colPivHouseholderQr().solve(b);
    //cout<<b<<endl;
    //cout<<A<<endl;
    //cout<<A*DeltaY<<endl;
    //cout<<A*DeltaY2<<endl;
    //cin.get();

    p.mPosition = p.mPosition + DeltaY.block<3,1>(0,0);
    p.mVelocity = p.mVelocity + DeltaY.block<3,1>(3,0);

    // p.mPosition[1] -= 0.005;
}


// RK4 integrator
void Simulator::RK4Solve(Particle& p) {
    // TODO: Replace the following code with midpoint method
    
    // double g= -9.81;

    // double k1p = mTimeStep * p.mVelocity[1];
    // double k1v = mTimeStep * g;
    // double k2p = mTimeStep * (p.mVelocity[1]+0.5*k1v);
    // double k2v = mTimeStep * g;
    // double k3p = mTimeStep * (p.mVelocity[1]+0.5*k2v);
    // double k3v = mTimeStep * g;
    // double k4p = mTimeStep * (p.mVelocity[1]+k3v);
    // double k4v = mTimeStep * g;

    // p.mPosition[1] = p.mPosition[1] + 1.0/6.0*k1p + 1.0/3.0*k2p + 1.0/3.0*k3p + 1.0/6.0*k4p;
    // p.mVelocity[1] = p.mVelocity[1] + 1.0/6.0*k1v + 1.0/3.0*k2v + 1.0/3.0*k3v + 1.0/6.0*k4v;

    // Note: RK4 integrator is accurate when compared with analytic solution

    ClearForce();
    ApplyForce();
    
    Eigen::Vector3d Pos0 = p.mPosition;
    Eigen::Vector3d Vel0 = p.mVelocity;

    ClearForce();
    ApplyForce();

    Eigen::Vector3d k1p = mTimeStep * p.mVelocity;
    Eigen::Vector3d k1v = mTimeStep * p.mAccumulatedForce/p.mMass; 

    p.mPosition = Pos0 + 0.5*k1p;
    p.mVelocity = Vel0 + 0.5*k1v;

    ClearForce();
    ApplyForce();

    Eigen::Vector3d k2p = mTimeStep * p.mVelocity;
    Eigen::Vector3d k2v = mTimeStep * p.mAccumulatedForce/p.mMass; 

    p.mPosition = Pos0 + 0.5*k2p;
    p.mVelocity = Vel0 + 0.5*k2v;

    ClearForce();
    ApplyForce();

    Eigen::Vector3d k3p = mTimeStep * p.mVelocity;
    Eigen::Vector3d k3v = mTimeStep * p.mAccumulatedForce/p.mMass; 

    p.mPosition = Pos0 + k3p;
    p.mVelocity = Vel0 + k3v;

    ClearForce();
    ApplyForce();

    Eigen::Vector3d k4p = mTimeStep * p.mVelocity;
    Eigen::Vector3d k4v = mTimeStep * p.mAccumulatedForce/p.mMass; 

    p.mPosition = Pos0 + 1.0/6.0*k1p + 1.0/3.0*k2p + 1.0/3.0*k3p + 1.0/6.0*k4p;
    p.mVelocity = Vel0 + 1.0/6.0*k1v + 1.0/3.0*k2v + 1.0/3.0*k3v + 1.0/6.0*k4v;

    // p.mPosition[1] -= 0.005;
}

void Simulator::InitVelocity()
{
    mInitVelocity.setZero();
    cout<<"Input initial vertical velocity: ";
    cin>>mInitVelocity[1];
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mVelocity = mInitVelocity;
    }
}

void Simulator::ClearForce()
{
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}

void Simulator::ApplyForce()
{
    if (sysType == 0)
    {
        ApplyForce_P1();
    }
    else
    {
        ApplyForce_P2();
    }
}

void Simulator::ApplyForce_P1()
{
    ApplyGravityForce();
    ApplyExtForce();
}

void Simulator::ApplyForce_P2()
{
    ApplyGravityForce();
    ApplyExtForce();
    ApplyConstraintForce();
}

void Simulator::ApplyExtForce()
{
    if (flag!=-1)
    {
        // cout<<"Apply external force"<<endl;
        mParticles[flag].mAccumulatedForce +=ExtForce; 
    }
    flag = -1;
    ExtForce.setZero();
}

void Simulator::ApplyGravityForce()
{
    for (unsigned int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] += -9.81 * mParticles[i].mMass;
    }
}

void Simulator::ApplyConstraintForce()
{
    unsigned int NumParticles = mParticles.size();
    Eigen::VectorXd q(3*NumParticles);
    Eigen::VectorXd q_deriv(3*NumParticles);
    Eigen::VectorXd Q(3*NumParticles);
    Eigen::MatrixXd W(3*NumParticles,3*NumParticles);
    Eigen::VectorXd lambda(2);
    Eigen::MatrixXd J(2,3*NumParticles);
    Eigen::MatrixXd J_deriv(2,3*NumParticles);
    Eigen::VectorXd C(2);
    double ks = 1.0;
    double kd = 1.25;

    q.setZero();
    q_deriv.setZero();
    Q.setZero();
    W.setZero();
    lambda.setZero();
    J.setZero();
    J_deriv.setZero();
    C.setZero();

    for (unsigned int i = 0; i<NumParticles; i++)
    {
        q.block<3,1>(3*i,0) = mParticles[i].mPosition;
        q_deriv.block<3,1>(3*i,0) = mParticles[i].mVelocity;
        Q.block<3,1>(3*i,0) = mParticles[i].mAccumulatedForce;
        W.block<3,3>(3*i,3*i) = 1/mParticles[i].mMass*Eigen::Matrix3d::Identity();
    }

    J.block<1,3>(0,0) = mParticles[0].mPosition.transpose();
    J.block<1,3>(1,0) = (mParticles[0].mPosition.transpose() - mParticles[1].mPosition.transpose()).eval();
    J.block<1,3>(1,3) = (mParticles[1].mPosition.transpose() - mParticles[0].mPosition.transpose()).eval();

    J_deriv.block<1,3>(0,0) = mParticles[0].mVelocity.transpose();
    J_deriv.block<1,3>(1,0) = (mParticles[0].mVelocity.transpose() - mParticles[1].mVelocity.transpose()).eval();
    J_deriv.block<1,3>(1,3) = (mParticles[1].mVelocity.transpose() - mParticles[0].mVelocity.transpose()).eval();

    C[0] = 0.5*mParticles[0].mPosition.transpose()*mParticles[0].mPosition-0.5*0.2*0.2;
    C[1] = 0.5*(mParticles[1].mPosition-mParticles[0].mPosition).transpose()*(mParticles[1].mPosition-mParticles[0].mPosition)-0.5*0.1*0.1;

    // lambda = ((J*W*J.transpose()).eval()).ldlt().solve((-J_deriv*q_deriv-J*W*Q-ks*C-kd*J*q_deriv).eval());
    // lambda = ((J*W*J.transpose()).eval()).colPivHouseholderQr().solve((-J_deriv*q_deriv-J*W*Q-ks*C-kd*J*q_deriv).eval());
    lambda = ((J*W*J.transpose()).eval()).jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve((-J_deriv*q_deriv-J*W*Q-ks*C-kd*J*q_deriv).eval());
    // lambda = ((J*W*J.transpose()).eval()).inverse()*((-J_deriv*q_deriv-J*W*Q-ks*C-kd*J*q_deriv).eval());

    Eigen::VectorXd Q_head(3*NumParticles);
    Q_head = (J.transpose()*lambda).eval();

    // cout<<"--------------------------"<<endl;
    // cout<<q<<endl;
    // cout<<q_deriv<<endl;
    // cout<<Q<<endl;
    // cout<<W<<endl;
    // cout<<J<<endl;
    // cout<<J_deriv<<endl;
    // cout<<C<<endl;
    // cout<<Q_head<<endl;
    // cout<<"##########################"<<endl;
    // cin.get();

    for (unsigned int i = 0; i<NumParticles; i++)
    {
        mParticles[i].mAccumulatedForce += Q_head.block<3,1>(3*i,0);
    }
}

void DerivativeEval()
{
// No need to compute DerivEval
}

//Part 1 : Galileo experiement
void Simulator::part1Galileo() {
    analyticSolve(mParticles[0]);
    fwdEulerSolve(mParticles[1]);
    midPointSolve(mParticles[2]);
    //ImplicitEurlerSolve(mParticles[3]);
    RK4Solve(mParticles[3]);
}

//Part 2 : Tinker Toy
void Simulator::part2TinkerToy() {
    // TODO: Replace the following code
    // for (unsigned int i = 0; i < mParticles.size(); i++) {
    //     mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    // }

    for (unsigned int i = 0; i < mParticles.size(); i++) {
        if (Integrator==0)
        {
            fwdEulerSolve(mParticles[i]);
        }
        else if (Integrator==1)
        {
            midPointSolve(mParticles[i]);
        }
        else if (Integrator==2)
        {
            RK4Solve(mParticles[i]);
        }
        else if (Integrator==3)
        {
            ImplicitEurlerSolve(mParticles[i]);
        }
    }

    // for (unsigned int i = 0; i < mParticles.size(); i++) {
    //     mParticles[i].mAccumulatedForce.setZero();
    // }
}



void Simulator::simulate() {
    if (sysType == 0) {
        part1Galileo();
    }
    else {
        part2TinkerToy();
    }
    mElapsedTime += mTimeStep;
    mFrameNumber++;
}







