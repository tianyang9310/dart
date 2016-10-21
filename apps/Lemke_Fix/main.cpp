#include <iostream>

#include "dart/dart.h"
#include "dart/lcpsolver/Lemke.h"

int main(int argc, char* argv[]) 
{
    Eigen::MatrixXd testA;
    testA.resize(1,1);
    testA<< 1;
    Eigen::VectorXd testb;
    testb.resize(1);
    testb<< -1.5;
    std::cout<<testA<<std::endl;
    std::cout<<testb<<std::endl;
    Eigen::VectorXd* f =  new Eigen::VectorXd(1);
    int err = dart::lcpsolver::Lemke(testA,testb,f);
    std::cout<<err<<std::endl;
    std::cout<<(*f)<<std::endl;
}

