#include "MyWorld.h"
#include "RigidBody.h"
#include <iostream>
#include "dart/lcpsolver/Lemke.h"

using namespace Eigen;
using namespace std;


#define COLLISION_EPSILON 1e-6

MyWorld::MyWorld() {
    mFrame = 0;
    mTimeStep = 0.001;
    mGravity = Vector3d(0.0, -9.8, 0.0);
    mForce.setZero();
    // Create a collision detector
    mCollisionDetector = new CollisionInterface();
    
    // Create and intialize two default rigid bodies
    //// RigidBody *rb1 = new RigidBody(dart::dynamics::Shape::BOX, Vector3d(0.05, 0.05, 0.05));
    //// mCollisionDetector->addRigidBody(rb1, "box"); // Put rb1 in collision detector
    //// rb1->mPosition[0] = -0.3;
    //// rb1->mPosition[1] = -0.5;
    //// 
    //// // rb1->mAngMomentum = Vector3d(0.0, 0.01, 0.0);
    //// mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(dart::dynamics::Shape::ELLIPSOID, Vector3d(0.06, 0.06, 0.06));
    mCollisionDetector->addRigidBody(rb2, "ellipse"); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.3;
    rb2->mPosition[1] = -0.5;
    rb2->mAngMomentum = Vector3d(0.01, 0.0, 0.0);
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);
    restingContact = false;
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
        // // update linear momentum and angular momentum according to result from collision handling
        // mRigidBodies[i]->mLinMomentum += mRigidBodies[i]->mAccumulatedLinImpulse;
        // mRigidBodies[i]->mAngMomentum += mRigidBodies[i]->mAccumulatedAngImpulse;

        // derivative of position and linear momentum
        Vector3d dPos = mRigidBodies[i]->mLinMomentum / mRigidBodies[i]->mMass;
        Vector3d dLinMom = mRigidBodies[i]->mMass * mGravity + mRigidBodies[i]->mAccumulatedForce;
        
        // derivation of angle and angular momentum
        mRigidBodies[i]->mQuatOrient.normalize();
        mRigidBodies[i]->mOrientation = mRigidBodies[i]->mQuatOrient.toRotationMatrix();
        Matrix3d curInertiaTensor = (mRigidBodies[i]->mOrientation * mRigidBodies[i]->mInertiaTensor * mRigidBodies[i]->mOrientation.transpose()).eval();
        Vector3d AngVel = curInertiaTensor.ldlt().solve(mRigidBodies[i]->mAngMomentum);
        Quaterniond QuatAngVel;
        QuatAngVel.w() = 0;
        QuatAngVel.vec() = AngVel;
        // ----------------------------
        Quaterniond dQuatOrient = QuatAngVel*mRigidBodies[i]->mQuatOrient;
        // Quaterniond dQuatOrient; 
        // dQuatOrient.w() = QuatAngVel.w()*mRigidBodies[i]->mQuatOrient.w() - QuatAngVel.vec().dot(mRigidBodies[i]->mQuatOrient.vec());
        // dQuatOrient.vec() = QuatAngVel.w()*mRigidBodies[i]->mQuatOrient.vec() + QuatAngVel.vec()*mRigidBodies[i]->mQuatOrient.w() + QuatAngVel.vec().cross(mRigidBodies[i]->mQuatOrient.vec());
        // ----------------------------
        dQuatOrient.w() = dQuatOrient.w() * 0.5;
        dQuatOrient.vec() = dQuatOrient.vec() * 0.5;
        Vector3d dAngMom = mRigidBodies[i]->mAccumulatedTorque;
        
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
        mRigidBodies[i]->mAccumulatedLinImpulse.setZero();
        mRigidBodies[i]->mAccumulatedAngImpulse.setZero();
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
    for (int i = 0; i < mRigidBodies.size(); i++) {
        // update linear momentum and angular momentum from collision handling
        mRigidBodies[i]->mLinMomentum += mRigidBodies[i]->mAccumulatedLinImpulse;
        mRigidBodies[i]->mAngMomentum += mRigidBodies[i]->mAccumulatedAngImpulse;
    }

    if (restingContact)
    {
        restingCollisionHandling();
    }
    
    // Break the pinata if it has enough momentum
    if (mPinataWorld->getSkeleton(0)->getCOMLinearVelocity().norm() > 0.6)
    mPinataWorld->getConstraintSolver()->removeAllConstraints();
}

// TODO: fill in the collision handling function
void MyWorld::collisionHandling() {
    // restitution coefficient
    double epsilon = 0.8;
    // double inf     = 1e100; // dInfinity; 
    // TODO: handle the collision events
    int numContacts = mCollisionDetector->getNumContacts();
    if (numContacts > 0)
    {
        // cout<<"There exist "<<numContacts<<" contact points."<<endl;
        for (int idxCnt=0; idxCnt<numContacts; idxCnt++)
        {
            // cout<<"The "<<(idxCnt+1)<<" contact point: "<<endl;
            RigidBody* rbA = mCollisionDetector->getContact(idxCnt).rb1;
            RigidBody* rbB = mCollisionDetector->getContact(idxCnt).rb2;
            Vector3d p = mCollisionDetector->getContact(idxCnt).point;  // contact coordinate
            Vector3d n = mCollisionDetector->getContact(idxCnt).normal; // contact normal

            // n = n / n.norm();
            
            // cout<<"Coordinate: "<<p.transpose()<<endl;
            // cout<<"Normal direction: "<<n.transpose()<<endl;
            
            double Ma_inv;
            double Mb_inv;
            Matrix3d Ic_a;
            Vector3d wa;
            Vector3d ra;
            Matrix3d Ic_b;
            Vector3d wb;
            Vector3d rb;
            Vector3d vr;
            
            if (rbA)
            {
                Ma_inv = 1.0/rbA->mMass;
                // rbA->mQuatOrient.normalize();
                rbA->mOrientation = rbA->mQuatOrient.toRotationMatrix();
                Ic_a = (rbA->mOrientation * rbA->mInertiaTensor * rbA->mOrientation.transpose()).eval();
                wa = Ic_a.ldlt().solve(rbA->mAngMomentum);
                ra = (p - rbA->mPosition).eval(); // contact point in ra frame
            }
            else
            {
                // one contact object is pinata
                // cout<<"Object A is pinata"<<endl;
                // Ma_inv = 0.0;  // infinite mass
                // Ic_a = Matrix3d::Identity()*inf;
                // ra = p; // contact point in pinata frame (a.k.a. world frame)
            }
            
            if (rbB)
            {
                Mb_inv = 1.0/rbB->mMass;
                // rbB->mQuatOrient.normalize();
                rbB->mOrientation = rbB->mQuatOrient.toRotationMatrix();
                Ic_b = (rbB->mOrientation * rbB->mInertiaTensor * rbB->mOrientation.transpose()).eval();
                wb = Ic_b.ldlt().solve(rbB->mAngMomentum);
                rb = (p - rbB->mPosition).eval(); // contact point in rb frame
            }
            else
            {
                // one contact object is pinata
                // cout<<"Object B is pinata"<<endl;
                // Mb_inv = 0.0;  // infinite mass
                // Ic_b = Matrix3d::Identity()*inf;
                // rb = p; // contact point in pinata frame (a.k.a. world frame)
            }
            
            if (rbA && rbB)
            {
                vr = (rbA->mLinMomentum*Ma_inv + wa.cross(ra)) - (rbB->mLinMomentum*Mb_inv+ wb.cross(rb)); // relative velocity (vector)
                // cout<<"Both rigid bodies are not pinata"<<endl;
            }
            else if (rbB)
            {
                vr = mCollisionDetector->getContact(idxCnt).pinataVelocity - (rbB->mLinMomentum*Mb_inv+ wb.cross(rb)); // relative velocity (vector)
                // cout<<"Rigid body B is not pinata"<<endl;
            }
            else
            {
                vr = (rbA->mLinMomentum*Ma_inv + wa.cross(ra)) - mCollisionDetector->getContact(idxCnt).pinataVelocity; // relative velocity (vector)
                // cout<<"Rigid body A is not pinata"<<endl;
            }
            
            double vrn= n.dot(vr); // normal relative velocity (scalar)
            double j;
            
            // cout<<vrn<<endl;
            if (vrn>=COLLISION_EPSILON)  // contact break away
            {
                break;
            }
            else if (vrn > -COLLISION_EPSILON) // resting contact
            {
                restingContact = true;
                mRestingContactList.push_back(mCollisionDetector->getContact(idxCnt));
                break; 
            }
            
            if (rbA && rbB)
            {
              j =                                             -(1+epsilon)*vrn/
                  ( Ma_inv + Mb_inv + n.dot((Ic_a.ldlt().solve(ra.cross(n))).cross(ra)) + n.dot((Ic_b.ldlt().solve(rb.cross(n))).cross(rb)) );
            }
            else if (rbB)
            {
              j =                                             -(1+epsilon)*vrn/
                ( Mb_inv + n.dot((Ic_b.ldlt().solve(rb.cross(n))).cross(rb)) );
            }
            else
            {
              j =                                             -(1+epsilon)*vrn/
                ( Ma_inv + n.dot((Ic_a.ldlt().solve(ra.cross(n))).cross(ra)) );
            }

            if (rbA)
            {
                rbA->mAccumulatedLinImpulse += j*n;
                rbA->mAccumulatedAngImpulse += ra.cross(j*n);
            }
            
            if (rbB)
            {
                rbB->mAccumulatedLinImpulse += -j*n;
                rbB->mAccumulatedAngImpulse += rb.cross(-j*n);
            }
        }
    }
}

void MyWorld::addObject(dart::dynamics::Shape::ShapeType _type, Vector3d _dim)
{
    // Legally spawn range
    // x: -0.3 0.3
    // y: -0.7 -0.3
    // z: -0.3 0.3
    RigidBody *newrb = new RigidBody(_type, _dim);
    if (_type == dart::dynamics::Shape::BOX) 
    {
        cout<<"Add a new cube"<<endl;
        mCollisionDetector->addRigidBody(newrb, "box"); 
    } 
    else if (_type == dart::dynamics::Shape::ELLIPSOID) 
    {
        cout<<"Add a new sphere"<<endl;
        mCollisionDetector->addRigidBody(newrb, "ellipse");
        newrb->mColor = Vector4d(0.2,0.8,0.2,1.0);
    }


    newrb->mPosition[0] = dart::math::random(-0.3,0.3);
    newrb->mPosition[1] = dart::math::random(-0.7,-0.3);
    newrb->mPosition[2] = dart::math::random(-0.3,0.3);

    newrb->mLinMomentum = 0.01*Vector3d::Random();
    newrb->mAngMomentum = 0.01*Vector3d::Random();
    
    mRigidBodies.push_back(newrb);

    // make sure the new object is collision free
    bool collision = false;
    for (size_t idx_Cnt=0; idx_Cnt<mCollisionDetector->getNumContacts(); idx_Cnt++)
    {
        if (mCollisionDetector->getContact(idx_Cnt).rb1 == newrb || 
            mCollisionDetector->getContact(idx_Cnt).rb2 == newrb)
        {
            collision = true;
            break; 
        }
    }
    
    if (collision)
    {
        mCollisionDetector->removeRigidBody(newrb);
        mRigidBodies.pop_back();
    }
}

void MyWorld::restingCollisionHandling()
{
    int numContacts = mRestingContactList.size();
    cout<<"Handling resting contact... there are "<<numContacts<<" resting contact points"<<endl;
    MatrixXd A(MatrixXd::Zero(numContacts,numContacts));
    VectorXd b(VectorXd::Zero(numContacts));
    VectorXd* f = new VectorXd(VectorXd::Zero(numContacts));

    compute_b(b);
    compute_A(A);

    int err = dart::lcpsolver::Lemke(A,b,f);
    for (size_t idx_RstCnt=0; idx_RstCnt<numContacts; idx_RstCnt++)
    {
        RigidContact ct = mRestingContactList[idx_RstCnt];
        if (ct.rb1)
        {
            ct.rb1->mAccumulatedForce += (*f)(idx_RstCnt)*ct.normal;
            ct.rb1->mAccumulatedTorque += (ct.point - ct.rb1->mPosition).cross((*f)(idx_RstCnt)*ct.normal);
        }
        if (ct.rb2)
        {
            ct.rb2->mAccumulatedForce += (*f)(idx_RstCnt)*ct.normal;
            ct.rb2->mAccumulatedTorque += (ct.point - ct.rb2->mPosition).cross((*f)(idx_RstCnt)*ct.normal);
        }
    
    }

    // clear Resting flag
    restingContact = false;
    mRestingContactList.clear();
}

void MyWorld::compute_A(Eigen::MatrixXd &A)
{
    int numContacts = mRestingContactList.size();
    for (size_t idx_RstCnt_i=0; idx_RstCnt_i<numContacts; idx_RstCnt_i++)
    {
        for (size_t idx_RstCnt_j=0; idx_RstCnt_j<numContacts; idx_RstCnt_j++)
        {
            A(idx_RstCnt_i,idx_RstCnt_j) = compute_aij(mRestingContactList[idx_RstCnt_i],mRestingContactList[idx_RstCnt_j]);
        }
    }
}

double MyWorld::compute_aij(RigidContact ct_i, RigidContact ct_j)
{
    if ((ct_i.rb1 != ct_j.rb1) && (ct_i.rb2 !=ct_j.rb2) && (ct_i.rb1 != ct_j.rb2) && (ct_i.rb2 != ct_j.rb1))
    {
        return 0.0;
    }

    RigidBody* rbA = ct_i.rb1;
    RigidBody* rbB = ct_i.rb2;
    Vector3d ni = ct_i.normal;
    Vector3d nj = ct_j.normal;
    Vector3d pi = ct_i.point;
    Vector3d pj = ct_j.point;
    Vector3d ra;
    Vector3d rb;
    Matrix3d Ic_a;
    Matrix3d Ic_b;

    if (rbA)
    {
        ra = (pi- rbA->mPosition).eval();
        rbA->mOrientation = rbA->mQuatOrient.toRotationMatrix();
        Ic_a = (rbA->mOrientation * rbA->mInertiaTensor * rbA->mOrientation.transpose()).eval().inverse();
    }
    else
    {
        ra = pi;
    }

    if (rbB)
    {
        rb = (pi- rbB->mPosition).eval();
        rbB->mOrientation = rbB->mQuatOrient.toRotationMatrix();
        Ic_b = (rbB->mOrientation * rbB->mInertiaTensor * rbB->mOrientation.transpose()).eval();
    }
    else
    {
        rb = pi;
    }

    Vector3d force_on_a = Vector3d::Zero();
    Vector3d torque_on_a = Vector3d::Zero();

    if (ct_j.rb1 == ct_i.rb1)
    {
        force_on_a = nj;
        if (rbA)
        {
            torque_on_a = (pj-rbA->mPosition).cross(nj);
        }
        else
        {
            torque_on_a = pj.cross(nj);
        }
    }
    else if (ct_j.rb2 == ct_i.rb1)
    {
        force_on_a = -nj;
        if (rbA)
        {
            torque_on_a = (pj-rbA->mPosition).cross(nj);
        }
        else
        {
            torque_on_a = pj.cross(nj);
        }
    }

    Vector3d force_on_b = Vector3d::Zero();
    Vector3d torque_on_b = Vector3d::Zero();
    if (ct_j.rb1 == ct_i.rb2)
    {
        force_on_b = nj;
        if (rbB)
        {
            torque_on_b = (pj-rbB->mPosition).cross(nj);
        }
        else
        {
            torque_on_b = pj.cross(nj);
        }
    }
    else if (ct_j.rb2 == ct_i.rb2)
    {
        force_on_b = -nj;
        if (rbB)
        {
            torque_on_b = (pj-rbB->mPosition).cross(nj);
        }
        else
        {
            torque_on_b = pj.cross(nj);
        }
    }

    Vector3d a_linear;
    Vector3d a_angular;
    if (rbA)
    {
        a_linear = force_on_a / rbA->mMass;
        a_angular = (Ic_a.ldlt().solve(torque_on_a)).cross(ra);
    }
    else
    {
        a_linear = Vector3d::Zero(); 
        a_angular =Vector3d::Zero();
    }


    Vector3d b_linear;
    Vector3d b_angular;
    if (rbB)
    {
        b_linear = force_on_b / rbB->mMass;
        b_angular = (Ic_b.ldlt().solve(torque_on_b)).cross(rb);
    }
    else
    {
        b_linear = Vector3d::Zero();
        b_angular = Vector3d::Zero();
    }
    
    return (ni.dot(((a_linear+a_angular) - (b_linear+b_angular))));
}

void MyWorld::compute_b(Eigen::VectorXd &b)
{
    int numContacts = mRestingContactList.size();
    for (size_t idx_RstCnt=0; idx_RstCnt<numContacts; idx_RstCnt++)
    {
        RigidContact ct = mRestingContactList[idx_RstCnt];
        RigidBody* rbA = ct.rb1;
        RigidBody* rbB = ct.rb2;
        Vector3d p = ct.point;
        Vector3d n = ct.normal;

        Matrix3d Ic_a;
        Matrix3d Ic_b;
        Vector3d ra;
        Vector3d rb;
        Vector3d wa;
        Vector3d wb;
        Vector3d vr;

        if (rbA)
        {
            ra = (p - rbA->mPosition).eval();
            rbA->mOrientation = rbA->mQuatOrient.toRotationMatrix();
            Ic_a = (rbA->mOrientation * rbA->mInertiaTensor * rbA->mOrientation.transpose()).eval().inverse();
            wa = Ic_a.ldlt().solve(rbA->mAngMomentum);
        }
        else
        {
            ra = p;
            wa = Vector3d::Zero();
        }

        if (rbB)
        {
            rb = (p - rbB->mPosition).eval();
            rbB->mOrientation = rbB->mQuatOrient.toRotationMatrix();
            Ic_b = (rbB->mOrientation * rbB->mInertiaTensor * rbB->mOrientation.transpose()).eval();
            wb = Ic_b.ldlt().solve(rbB->mAngMomentum);
        }
        else
        {
            rb = p;
            wb = Vector3d::Zero();
        }

        // Supposing we have fixed pinata
        // And rigid body B would always be pinata if it is involved in this contact
        // get the external forces and torques
        Vector3d f_ext_a;
        Vector3d f_ext_b;
        if (rbA)
        {
            f_ext_a = mGravity * rbA->mMass;
        }
        else
        {
            f_ext_a = Vector3d::Zero();
        }
        if (rbB)
        {
            f_ext_b = mGravity * rbB->mMass;
        }
        else
        {
            f_ext_b = Vector3d::Zero();
        }
        Vector3d t_ext_a = Vector3d::Zero();
        Vector3d t_ext_b = Vector3d::Zero();


        Vector3d a_ext_part;
        Vector3d b_ext_part;
        Vector3d a_vel_part;
        Vector3d b_vel_part;

        if (rbA)
        {
            a_ext_part = f_ext_a / rbA->mMass + ((Ic_a.ldlt().solve(t_ext_a)).cross(ra));
            a_vel_part = wa.cross(wa.cross(ra)) + ((Ic_a.ldlt().solve(rbA->mAngMomentum.cross(wa))).cross(ra));
        }
        else
        {
            a_ext_part = Vector3d::Zero();
            a_vel_part = Vector3d::Zero();
        }
        if (rbB)
        {
            b_ext_part = f_ext_b / rbB->mMass + ((Ic_b.ldlt().solve(t_ext_b)).cross(rb));
            b_vel_part = wb.cross(wb.cross(rb)) + ((Ic_b.ldlt().solve(rbB->mAngMomentum.cross(wb))).cross(rb));
        }
        else
        {
            b_ext_part = Vector3d::Zero();
            b_vel_part = Vector3d::Zero();
        }


        double k1 = n.dot((a_ext_part+a_vel_part)-(b_ext_part+b_vel_part));
        Vector3d ndot = wb.cross(n);

        if (rbA && rbB)
        {
            vr = (rbA->mLinMomentum/rbA->mMass + wa.cross(ra)) - (rbB->mLinMomentum/rbB->mMass+ wb.cross(rb)); 
        }
        else if (rbB)
        {
            vr = - (rbB->mLinMomentum/rbB->mMass+ wb.cross(rb));
        }
        else
        {
            vr = (rbA->mLinMomentum/rbA->mMass + wa.cross(ra));
        }

        double k2 = 2 * ndot.dot(vr);

        b[idx_RstCnt] = k1 + k2;
    }

}
