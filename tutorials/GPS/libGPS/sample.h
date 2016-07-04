#ifndef SAMPLE_H
#define SAMPLE_H

#include <vector>
#include <Eigen/Eigen>

namespace GPS_NSpace
{

using namespace std;
using namespace Eigen;

class sample
{
public:
	MatrixXd x;
	MatrixXd u;
    vector<MatrixXd> Quu_inv;
    VectorXd Logq;
};



}
#endif
