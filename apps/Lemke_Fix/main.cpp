#include <iostream>

#include "dart/dart.h"
#include "MyLemke.h"
// #include "dart/lcpsolver/Lemke.h"

int main(int argc, char* argv[]) 
{
    Eigen::MatrixXd testA;
    Eigen::VectorXd testb;
    Eigen::VectorXd* f;
    int err;

    f =  new Eigen::VectorXd(1);
    testA.resize(1,1);
    testA << 1;
    testb.resize(1);
    testb<< -1.5;
    std::cout<<"Matrix A is"<<std::endl;
    std::cout<<testA<<std::endl;
    std::cout<<"Vector b is"<<std::endl;
    std::cout<<testb<<std::endl;
    err = dart::lcpsolver::YT::Lemke(testA,testb,f);
    std::cout<<"Lemke solution z is"<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"Lemke error type is "<<err<<std::endl;

    std::cout<<std::endl<<std::endl;
    std::cout<<"-----Validation-----"<<std::endl;
    std::cout<<"z: "<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"A*z+b: "<<std::endl;
    std::cout<<testA*(*f) + testb<<std::endl;

    std::cout<<"=========================================="<<std::endl;
    f =  new Eigen::VectorXd(2);
    testA.resize(2,2);
    testA << 3.12, 0.1877, 0.1877, 3.254;
    testb.resize(2);
    testb<< -0.00662, -0.006711;
    std::cout<<"Matrix A is"<<std::endl;
    std::cout<<testA<<std::endl;
    std::cout<<"Vector b is"<<std::endl;
    std::cout<<testb<<std::endl;
    err = dart::lcpsolver::YT::Lemke(testA,testb,f);
    std::cout<<"Lemke solution z is"<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"Lemke error type is "<<err<<std::endl;

    std::cout<<std::endl<<std::endl;
    std::cout<<"-----Validation-----"<<std::endl;
    std::cout<<"z: "<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"A*z+b: "<<std::endl;
    std::cout<<testA*(*f) + testb<<std::endl;

    std::cout<<"=========================================="<<std::endl;
    f =  new Eigen::VectorXd(2);
    testA.resize(2,2);
    testA <<4.001 ,  -2 ,-2,3.999 ;
    testb.resize(2);
    testb<< -0.009819, -0.009798;
    std::cout<<"Matrix A is"<<std::endl;
    std::cout<<testA<<std::endl;
    std::cout<<"Vector b is"<<std::endl;
    std::cout<<testb<<std::endl;
    err = dart::lcpsolver::YT::Lemke(testA,testb,f);
    std::cout<<"Lemke solution z is"<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"Lemke error type is "<<err<<std::endl;

    std::cout<<std::endl<<std::endl;
    std::cout<<"-----Validation-----"<<std::endl;
    std::cout<<"z: "<<std::endl;
    std::cout<<(*f)<<std::endl;
    std::cout<<"A*z+b: "<<std::endl;
    std::cout<<testA*(*f) + testb<<std::endl;
}

