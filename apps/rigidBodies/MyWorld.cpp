#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include <iostream>

using namespace Eigen;
using namespace std;

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();
    
    // Create and intialize two default rigid bodies
    RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    rb1->mPosition[0] = -0.3;
    rb1->mPosition[1] = -0.5;
    
    rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
}

void MyWorld::initializePinata() {
    // Add pinata to the collison detector
    mCollisionDetector->addSkeleton(mPinataWorld->getSkeleton(0));
    
    // Add some damping in the Pinata joints
    int nJoints = mPinataWorld->getSkeleton(0)->getNumBodyNodes();
    for (int i = 0; i < nJoints; i++) {
        int nDofs = mPinataWorld->getSkeleton(0)->getJoint(i)->getNumDofs();
        for (int j = 0; j < nDofs; j++)
        mPinataWorld->getSkeleton(0)->getJoint(i)->setDampingCoefficient(j, 1.0);
    }
    
    // Weld two seems to make a box
    dart::dynamics::BodyNode* top = mPinataWorld->getSkeleton(0)->getBodyNode("top");
    dart::dynamics::BodyNode* front = mPinataWorld->getSkeleton(0)->getBodyNode("front");
    dart::dynamics::BodyNode* back = mPinataWorld->getSkeleton(0)->getBodyNode("back");
    dart::constraint::WeldJointConstraint *joint1 = new dart::constraint::WeldJointConstraint(top, front);
    dart::constraint::WeldJointConstraint *joint2 = new dart::constraint::WeldJointConstraint(top, back);
    mPinataWorld->getConstraintSolver()->addConstraint(joint1);
    mPinataWorld->getConstraintSolver()->addConstraint(joint2);
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
    delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
    delete mCollisionDetector;
}

void MyWorld::simulate() {
    mFrame++;
    
    // TODO: The skeleton code has provided the integration of position and linear momentum,
    // your first job is to fill in the integration of orientation and angular momentum.
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // derivative of position and linear momentum
        Eigen::Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Eigen::Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;
        
        // derivation of angle and angular momentum
        mRigidBodies[i]->mOrientation = mRigidBodies[i]->mQuatOrient.toRotationMatrix();
        Eigen::Matrix3d curInertiaTensor = (mRigidBodies[i]->mOrientation * mRigidBodies[i]->mInertiaTensor * mRigidBodies[i]->mOrientation.transpose()).eval();
        Eigen::Vector3d AngVel = curInertiaTensor.ldlt().solve(mRigidBodies[i]->mAngMomentum);
        Eigen::Quaterniond QuatAngVel;
        QuatAngVel.w() = 0;
        QuatAngVel.vec() = AngVel;
        Eigen::Quaterniond dQuatOrient = QuatAngVel*mRigidBodies[i]->mQuatOrient;
        dQuatOrient.w() = dQuatOrient.w() * 0.5;
        dQuatOrient.vec() = dQuatOrient.vec() * 0.5;
        Eigen::Vector3d dAngMom = mRigidBodies[i]->mAccumulatedTorque;
        
        // update position and linear momentum
        mRigidBodies[i]->mPosition += dPos * mTimeStep;
        mRigidBodies[i]->mLinMomentum += mTimeStep * dLinMom;
        
        // update angle and angular momentum
        mRigidBodies[i]->mQuatOrient.w() += mTimeStep * dQuatOrient.w();
        mRigidBodies[i]->mQuatOrient.vec() += mTimeStep * dQuatOrient.vec();
        mRigidBodies[i]->mQuatOrient.normalize();   // Normalize to ensure effective quaternion whose norm is 1
        mRigidBodies[i]->mAngMomentum += mTimeStep * dAngMom;
    }
    
    // Reset accumulated force and torque to be zero after a complete integration
    for (int i = 0; i < mRigidBodies.size(); i++) {
        mRigidBodies[i]->mAccumulatedForce.setZero();
        mRigidBodies[i]->mAccumulatedTorque.setZero();
    }
    
    // Apply external force to the pinata
    mPinataWorld->getSkeleton(0)->getBodyNode("bottom")->addExtForce(mForce);
    mForce.setZero();
    
    // Simulate Pinata using DART
    mPinataWorld->step();
    
    // Run collision detector
    mCollisionDetector->checkCollision();
    
    // TODO: implement a collision handler
    collisionHandling();
    
    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;
    double inf     = 1e10;
    
    // TODO: handle the collision events
    int numContacts = mCollisionDetector->getNumContacts();
    if (numContacts > 0)
    {
        cout<<"There exist "<<numContacts<<" contact points."<<endl;
        for (int idxCnt=0; idxCnt<numContacts; idxCnt++)
        {
            cout<<"The "<<(idxCnt+1)<<" contact point: "<<endl;
            RigidBody* rbA = mCollisionDetector->getContact(idxCnt).rb1;
            RigidBody* rbB = mCollisionDetector->getContact(idxCnt).rb2;
            Eigen::Vector3d p = mCollisionDetector->getContact(idxCnt).point;  // contact coordinate
            Eigen::Vector3d n = mCollisionDetector->getContact(idxCnt).normal; // contact normal
            
            cout<<"Coordinate: "<<p.transpose()<<endl;
            cout<<"Normal direction: "<<n.transpose()<<endl;
            
            double Ma_inv;
            double Mb_inv;
            Eigen::Matrix3d Ic_a;
            Eigen::Vector3d wa;
            Eigen::Vector3d ra;
            Eigen::Matrix3d Ic_b;
            Eigen::Vector3d wb;
            Eigen::Vector3d rb;
            Eigen::Vector3d vr;
            
            if (rbA)
            {
                Ma_inv = 1/rbA->mMass;
                rbA->mOrientation = rbA->mQuatOrient.toRotationMatrix();
                Ic_a = (rbA->mOrientation * rbA->mInertiaTensor * rbA->mOrientation.transpose()).eval();
                wa = Ic_a.ldlt().solve(rbA->mAngMomentum);
                ra = (p - rbA->mPosition).eval(); // contact point in ra frame
            }
            else
            {
                // one contact object is pinata
                cout<<"Object A is pinata"<<endl;
                Ma_inv = 0.0;  // infinite mass
                Ic_a = Eigen::Matrix3d::Identity()*inf;
                ra = p; // contact point in pinata frame (a.k.a. world frame)
            }
            
            if (rbB)
            {
                Mb_inv = 1/rbB->mMass;
                rbB->mOrientation = rbB->mQuatOrient.toRotationMatrix();
                Ic_b = (rbB->mOrientation * rbB->mInertiaTensor * rbB->mOrientation.transpose()).eval();
                wb = Ic_b.ldlt().solve(rbB->mAngMomentum);
                rb = (p - rbB->mPosition).eval(); // contact point in rb frame
            }
            else
            {
                // one contact object is pinata
                cout<<"Object B is pinata"<<endl;
                Mb_inv = 0.0;  // infinite mass
                Ic_b = Eigen::Matrix3d::Identity()*inf;
                rb = p; // contact point in pinata frame (a.k.a. world frame)
            }
            
            if (rbA && rbB)
            {
                vr = (rbA->mLinMomentum*Ma_inv + wa.cross(ra)) - (rbB->mLinMomentum*Mb_inv+ wb.cross(rb)); // relative velocity (vector)
            }
            else if (rbB)
            {
                vr = mCollisionDetector->getContact(idxCnt).pinataVelocity - (rbB->mLinMomentum*Mb_inv+ wb.cross(rb)); // relative velocity (vector)
            }
            else
            {
                vr = (rbA->mLinMomentum*Ma_inv + wa.cross(ra)) - mCollisionDetector->getContact(idxCnt).pinataVelocity; // relative velocity (vector)
            }
            
            double vrn= n.dot(vr); // normal relative velocity (scalar)
            double j;
            
            j =                                             -(1+epsilon)*vrn/
                ( Ma_inv + Mb_inv + n.dot((Ic_a.ldlt().solve(ra.cross(n))).cross(ra)) + n.dot((Ic_b.ldlt().solve(rb.cross(n))).cross(rb)) );
            
            if (rbA)
            {
                rbA->mLinMomentum += j*n;
                rbA->mAngMomentum += ra.cross(j*n);
            }
            
            if (rbB)
            {
                rbB->mLinMomentum += -j*n;
                rbB->mAngMomentum += rb.cross(-j*n);
            }
        }
    }
}
