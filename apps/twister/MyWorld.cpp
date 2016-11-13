#include "MyWorld.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
  // Load a skeleton from file
  mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");

  // Create markers
  createMarkers();
  
  // Initialize Jacobian assuming that there is only one constraint
  // mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
  // mConstrainedMarker = -1;
  leftRight = false;
  initPos = mSkel->getPositions();
  initVel = mSkel->getVelocities();
  objective = false;
}

MyWorld::~MyWorld() {
}

bool MyWorld::getObjective(){
    return objective;
}

void MyWorld::setObjective(bool _objective){
    objective = _objective;
    if (objective) {
        dtmsg<<"[Add] Objective."<<std::endl;
    } else {
        dtmsg<<"[Remove] Objective."<<std::endl;
    }
}

bool MyWorld::getLeftRight(){
    return leftRight;
}
void MyWorld::setLeftRight(bool _leftRight){
    leftRight = _leftRight;
    if (leftRight) {
        dtmsg<<"[Add] Left and right constraint."<<std::endl;
        Marker* leftMarker = mSkel->getMarker("left_hand");
        Marker* rightMarker = mSkel->getMarker("right_hand");
        bool found = false;
        for (auto it = mMarkerTargetBundle.begin(); it < mMarkerTargetBundle.end(); it++){
            if (mMarkers[(*it).first] == leftMarker){
                found = true;
                break;
            } else if (mMarkers[(*it).first] == rightMarker){
                found = true;
                break;
            }
        }
        if (found) {
            dtwarn<<"Current constraint group contains left hand or right hand. Thus the two hands may not exactly overlap."<<std::endl;
        }
    } else {
        dtmsg<<"[Remove] Left and right hand constraint."<<std::endl;
    }
}
void MyWorld::reset(){
    mSkel->setPositions(initPos);
    mSkel->setVelocities(initVel);
    leftRight = false;
    mMarkerTargetBundle.clear();
}

void MyWorld::solve() {
    if (mMarkerTargetBundle.empty() && !leftRight && !objective){
        return;
    } 
//  if (mConstrainedMarker == -1)
//    return; 
  int numIter = 300;
  double alpha = 0.01;
  int nDof = mSkel->getNumDofs();
  VectorXd gradients(nDof);
  VectorXd newPose(nDof);
  for (int i = 0; i < numIter; i++) {
    gradients.setZero();
    for (int cnstridx = 0 ; cnstridx < mMarkerTargetBundle.size(); cnstridx++){
        gradients += updateGradients(mMarkerTargetBundle[cnstridx]);
    }
    if (leftRight) {
        gradients += updateGradientsLeftRightHand();
    }
    if (objective) {
        gradients += updateGradientsObjective();
    }
    newPose = mSkel->getPositions() - alpha * gradients;
    mSkel->setPositions(newPose); 
    mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
  }
  
}

void MyWorld::computeJacobian(Eigen::Vector4d offset, BodyNode* node, Eigen::MatrixXd& mJ){
    mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
    while (node->getParentBodyNode() != nullptr){
        Joint *joint = node->getParentJoint();
        Matrix4d world2Parent = node->getParentBodyNode()->getTransform().matrix();
        Matrix4d parent2Joint = joint->getTransformFromParentBodyNode().matrix();
        std::vector<Matrix4d> jointDofBundle(joint->getNumDofs());
        Matrix4d joint2Child = joint->getTransformFromChildBodyNode().inverse().matrix();
        for (int d_dofidx = 0; d_dofidx < joint->getNumDofs(); d_dofidx++){
            jointDofBundle[d_dofidx] = joint->getTransformDerivative(d_dofidx);
            for (int dofidx = 0; dofidx < joint->getNumDofs(); dofidx++){
                if (dofidx == d_dofidx){
                    continue;
                }
                jointDofBundle[dofidx] = joint->getTransform(dofidx).matrix();
            }
            MatrixXd jColBeforeHand = world2Parent * parent2Joint;
            for (int dofidx = 0; dofidx < joint->getNumDofs(); dofidx++){
                jColBeforeHand = jColBeforeHand * jointDofBundle[dofidx];
            }
            VectorXd jCol = jColBeforeHand * joint2Child * offset;
            int colIndex = joint->getIndexInSkeleton(d_dofidx);
            mJ.col(colIndex) = jCol.head(3);
        }

        offset = node->getTransform(node->getParentBodyNode()).matrix() * offset;
        // offset = joint2Child * offset;
        // for (int dofidx = joint->getNumDofs()-1; dofidx >= 0; dofidx--){
        //     offset = joint->getTransform(dofidx).matrix() * offset; 
        // }
        // offset = parent2Joint * offset;
        
        node = node->getParentBodyNode();
    }
}

VectorXd MyWorld::updateGradientsLeftRightHand(){
    Marker* leftMarker = mSkel->getMarker("left_hand");
    Marker* rightMarker = mSkel->getMarker("right_hand");
    BodyNode* leftBodyNode = mSkel->getBodyNode("h_hand_left");
    BodyNode* rightBodyNode = mSkel->getBodyNode("h_hand_right");

    // compute constraint
    Vector3d mC;
    mC = leftMarker->getWorldPosition() - rightMarker->getWorldPosition();

    // compute jacobian
    MatrixXd left_mJ;
    Vector4d left_offset;
    left_offset << leftMarker->getLocalPosition(), 1;
    computeJacobian(left_offset, leftBodyNode, left_mJ);

    MatrixXd right_mJ;
    Vector4d right_offset;
    right_offset << rightMarker->getLocalPosition(), 1;
    computeJacobian(right_offset, rightBodyNode, right_mJ);

    MatrixXd mJ = left_mJ - right_mJ;
    double wi = 10.0; // this weight can stress constraint on left and right hand.
    VectorXd gradients = 2 * wi * mJ.transpose() * mC;
    return gradients;
}

VectorXd MyWorld::updateGradientsObjective(){
    double wi = 1.0;
    VectorXd gradients = 2 * wi * (mSkel->getPositions() - initPos);
    return gradients;
}

// Current code only works for the left leg with only one constraint
VectorXd MyWorld::updateGradients(std::pair<int,Eigen::VectorXd> _MarkerTarget) {
    int markeridx = _MarkerTarget.first;
    Vector3d markertarget = _MarkerTarget.second;
    // compute constraint
    Vector3d mC;
    mC = getMarker(markeridx)->getWorldPosition() - markertarget;

    // compute jacobian
    MatrixXd mJ;
    Vector4d offset;
    offset << getMarker(markeridx)->getLocalPosition(), 1;
    BodyNode *node = getMarker(markeridx)->getBodyNode();
    computeJacobian(offset,node,mJ);

    VectorXd gradients = 2 * mJ.transpose() * mC;
    return gradients;
/*
  // compute c(q)
  mC = getMarker(mConstrainedMarker)->getWorldPosition() - mTarget;

  // compute J(q)
  Vector4d offset;
  offset << getMarker(mConstrainedMarker)->getLocalPosition(), 1; // Create a vector in homogeneous coordinates
  // w.r.t ankle dofs
  BodyNode *node = getMarker(mConstrainedMarker)->getBodyNode();
  Joint *joint = node->getParentJoint();
  Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
  Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  Matrix4d dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  Matrix4d R = joint->getTransform(1).matrix();
  Matrix4d jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  Vector4d jCol = worldToParent * parentToJoint * dR * R * jointToChild * offset;
  int colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of jCol
  dR = joint->getTransformDerivative(1);
  R = joint->getTransform(0).matrix();
  jCol = worldToParent * parentToJoint * R * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(1);
  mJ.col(colIndex) = jCol.head(3);
  offset = parentToJoint * joint->getTransform(0).matrix() * joint->getTransform(1).matrix() * jointToChild * offset; // Update offset so it stores the chain below the parent joint

  // w.r.t knee dof
  node = node->getParentBodyNode(); // return NULL if node is the root node
  joint = node->getParentJoint();
  worldToParent = node->getParentBodyNode()->getTransform().matrix();
  parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  jCol = worldToParent * parentToJoint * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of jCol
  offset = parentToJoint * joint->getTransform(0).matrix() * jointToChild * offset;

  // w.r.t hip dofs
  node = node->getParentBodyNode();
  joint = node->getParentJoint();
  worldToParent = node->getParentBodyNode()->getTransform().matrix();
  parentToJoint = joint->getTransformFromParentBodyNode().matrix();
  dR = joint->getTransformDerivative(0); // Doesn't need .matrix() because it returns a Matrix4d instead of Isometry3d
  Matrix4d R1 = joint->getTransform(1).matrix();
  Matrix4d R2 = joint->getTransform(2).matrix();
  jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();
  jCol = worldToParent * parentToJoint * dR * R1 * R2 * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(0);
  mJ.col(colIndex) = jCol.head(3); // Take the first 3 elelemtns of J

  R1 = joint->getTransform(0).matrix();
  dR = joint->getTransformDerivative(1);
  R2 = joint->getTransform(2).matrix();
  jCol = worldToParent * parentToJoint * R1 * dR * R2 * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(1);
  mJ.col(colIndex) = jCol.head(3);

  R1 = joint->getTransform(0).matrix();
  R2 = joint->getTransform(1).matrix();
  dR = joint->getTransformDerivative(2);
  jCol = worldToParent * parentToJoint * R1 * R2 * dR * jointToChild * offset;
  colIndex = joint->getIndexInSkeleton(2);
  mJ.col(colIndex) = jCol.head(3);

  // compute gradients
  VectorXd gradients = 2 * mJ.transpose() * mC;
  return gradients;
*/
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index) {
    bool found = false;
    for (auto it = mMarkerTargetBundle.begin(); it < mMarkerTargetBundle.end(); it++){
        if ((*it).first == _index){
            found = true;
            break;
        }
    }
    if (!found){
        Vector3d target = getMarker(_index)->getWorldPosition();
        mMarkerTargetBundle.push_back(std::make_pair(_index,target));
        dtmsg<<"[Add] "<<getMarker(_index)->getName()<<" constraint as "<<target.transpose()<<std::endl;
    } else {
        dtwarn<<"[Warning] already have this constraint!"<<std::endl;
    }

//  if (_index == 0) {
//    mTarget = getMarker(_index)->getWorldPosition();
//    mConstrainedMarker = _index;
//  } else {
//    mConstrainedMarker = -1;
//  }
}

void MyWorld::modifyConstraint(int _index, Vector3d _deltaP) {
    for (auto it = mMarkerTargetBundle.begin(); it < mMarkerTargetBundle.end(); it++){
        if ((*it).first == _index){
            (*it).second += _deltaP;
            dtmsg<<"[Modify] "<<getMarker(_index)->getName()<<" constraint as "<<(*it).second.transpose()<<std::endl;
            break;
        }
    }
//  if (mConstrainedMarker == 0)
//    mTarget += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
    for (auto it = mMarkerTargetBundle.begin(); it < mMarkerTargetBundle.end(); it++){
        if ((*it).first == _index){
            dtmsg<<"[Remove] "<<getMarker(_index)->getName()<<" constraint."<<std::endl;
            mMarkerTargetBundle.erase(it);
            break;
        }
    }
// mConstrainedMarker = -1;
}

Marker* MyWorld::getMarker(int _index) {
  return mMarkers[_index];
}

std::vector<std::pair<int,Eigen::Vector3d>> MyWorld::getMarkerTargetBundle(){
  return mMarkerTargetBundle;
}

void MyWorld::createMarkers() {
  Vector3d offset(0.2, 0.0, 0.0);
  BodyNode* bNode = mSkel->getBodyNode("h_heel_right");
  Marker* m = new Marker("right_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.2, 0.0, 0.0);
  bNode = mSkel->getBodyNode("h_heel_left");
  m = new Marker("left_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_right");
  m = new Marker("right_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_left");
  m = new Marker("left_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, 0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, -0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.075, 0.1, 0.0);
  bNode = mSkel->getBodyNode("h_abdomen");
  m = new Marker("abdomen", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, 0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, -0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_right");
  m = new Marker("right_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_left");
  m = new Marker("left_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, 0.05);
  bNode = mSkel->getBodyNode("h_bicep_right");
  m = new Marker("right_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, -0.05);
  bNode = mSkel->getBodyNode("h_bicep_left");
  m = new Marker("left_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, 0.025);
  bNode = mSkel->getBodyNode("h_hand_right");
  m = new Marker("right_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, -0.025);
  bNode = mSkel->getBodyNode("h_hand_left");
  m = new Marker("left_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);
}
