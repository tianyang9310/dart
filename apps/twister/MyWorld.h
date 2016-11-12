#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include <utility>
#include "dart/dart.h"


class MyWorld {
 public:
    MyWorld();
    virtual ~MyWorld();
    dart::dynamics::SkeletonPtr getSkel() {
        return mSkel;
    }

    void solve();
    void createConstraint(int _index);
    void modifyConstraint(int _index, Eigen::Vector3d _deltaP);
    void removeConstraint(int _index);
    dart::dynamics::Marker* getMarker(int _index);

 protected:
    Eigen::VectorXd updateGradients(std::pair<int,Eigen::VectorXd> _MarkerTarget);
    void createMarkers();

    dart::dynamics::SkeletonPtr mSkel;
    std::vector<dart::dynamics::Marker*> mMarkers;
    Eigen::Vector3d mC;
    Eigen::MatrixXd mJ;
    // Eigen::Vector3d mTarget; // The target location of the constriant
    // int mConstrainedMarker; // The index of the constrained marker
    std::vector<std::pair<int,Eigen::Vector3d>> mMarkerTargetBundle;
};

#endif
