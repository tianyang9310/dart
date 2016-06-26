#ifndef POLICYOPTCAFFE_H
#define POLICYOPTCAFFE_H

#include <caffe/caffe.hpp>

namespace GPS_NSpace
{
using namespace caffe;

class PolicyOptCaffe
{
public:
    PolicyOptCaffe();
    void InitSolver();
// ----------------------------------
//  necessary var for NN initialization
    int batch_size;

}

}

#endif
