#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include <Eigen/Dense>
#include "dart/dart.h"

class RigidBody {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	RigidBody(dart::dynamics::Shape::ShapeType _type, Eigen::Vector3d _dim) {
        // Create a default rigid body
        mMass = 1.0;
        mPosition.setZero(); // x = (0, 0, 0)
        mOrientation.setIdentity(); // R = identity
		mQuatOrient.setIdentity();	//initialize quaternion
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
        mInertiaTensor.setIdentity();
        
        if (_type == dart::dynamics::Shape::BOX) {
            mShape = Eigen::make_aligned_shared<dart::dynamics::BoxShape>(_dim);
            mInertiaTensor(0,0) = 1.0/12.0*(_dim(1)*_dim(1)+_dim(2)*_dim(2))*mMass;
            mInertiaTensor(1,1) = 1.0/12.0*(_dim(0)*_dim(0)+_dim(2)*_dim(2))*mMass;
            mInertiaTensor(2,2) = 1.0/12.0*(_dim(0)*_dim(0)+_dim(1)*_dim(1))*mMass;
        } else if (_type == dart::dynamics::Shape::ELLIPSOID) {
            mShape = Eigen::make_aligned_shared<dart::dynamics::EllipsoidShape>(_dim);
            mInertiaTensor(0,0) = 1.0/5.0*(_dim(1)*_dim(1)+_dim(2)*_dim(2))*mMass;
            mInertiaTensor(1,1) = 1.0/5.0*(_dim(0)*_dim(0)+_dim(2)*_dim(2))*mMass;
            mInertiaTensor(2,2) = 1.0/5.0*(_dim(0)*_dim(0)+_dim(1)*_dim(1))*mMass;
        }
        
        mLinMomentum.setZero();
        mAngMomentum.setZero();
        
        mAccumulatedForce.setZero();
        mAccumulatedTorque.setZero();
        mAccumulatedLinImpulse.setZero();
        mAccumulatedAngImpulse.setZero();
    }
    virtual ~RigidBody() {}

    void draw(dart::renderer::RenderInterface* _ri);

    int getConfigSize() {
		return mPosition.size() + mOrientation.size();
    }
    
    double mMass;

	Eigen::Vector3d mPosition;
    Eigen::Quaterniond mQuatOrient; // quaternion
	Eigen::Matrix3d mOrientation;   // rotation matrix
    Eigen::Matrix3d mInertiaTensor;
    Eigen::Vector3d mLinMomentum;
    Eigen::Vector3d mAngMomentum;
    dart::dynamics::ShapePtr mShape;
    
	Eigen::Vector3d mAccumulatedForce;
    Eigen::Vector3d mAccumulatedTorque;
	Eigen::Vector3d mAccumulatedLinImpulse;
    Eigen::Vector3d mAccumulatedAngImpulse;

    Eigen::Vector4d mColor;
};

#endif
