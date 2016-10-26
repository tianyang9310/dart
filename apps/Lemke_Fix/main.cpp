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

    std::cout<<"=========================================="<<std::endl;
    f =  new Eigen::VectorXd(4);
    testA.resize(4,4);
    testA <<
 3.999,0.9985, 1.001,    -2,
0.9985, 3.998,    -2,0.9995,
 1.001,    -2, 4.002, 1.001,
    -2,0.9995, 1.001, 4.001;



    testb.resize(4);
    testb<< 
 -0.01008,
-0.009494,
 -0.07234,
 -0.07177;

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
    f =  new Eigen::VectorXd(6);
    testA.resize(6,6);
    testA <<
    3.1360,   -2.0370,   0.9723,   0.1096,  -2.0370,   0.9723,
   -2.0370,    3.7820,   0.8302,  -0.0257,   2.4730,   0.0105,
    0.9723,    0.8302,   5.1250,  -2.2390,  -1.9120,   3.4080,
    0.1096,   -0.0257,  -2.2390,   3.1010,  -0.0257,  -2.2390,
   -2.0370,    2.4730,  -1.9120,  -0.0257,   5.4870,  -0.0242,
    0.9723,    0.0105,   3.4080,  -2.2390,  -0.0242,   3.3860;



    testb.resize(6);
    testb<<
    0.1649,
   -0.0025,
   -0.0904,
   -0.0093,
   -0.0000,
   -0.0889;

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
    // TODO: find out why this high dimension case not make sense
    f =  new Eigen::VectorXd(12);
    testA.resize(12,12);
    testA <<
   4.03, -1.014, -1.898,   1.03, -1.014, -1.898,      1, -1.014, -1.898,     -2, -1.014, -1.898,
 -1.014,  4.885, -1.259,  1.888,   3.81,  2.345, -1.879,  1.281, -2.334,  1.022,  0.206,   1.27,
 -1.898, -1.259,    3.2, -1.032,-0.6849,  1.275,  1.003, 0.6657,  3.774,  1.869,   1.24,   1.85,
   1.03,  1.888, -1.032,   4.03,  1.888, -1.032,     -2,  1.888, -1.032,      1,  1.888, -1.032,
 -1.014,   3.81,-0.6849,  1.888,  3.225,  1.275, -1.879,   1.85,  -1.27,  1.022,  1.265, 0.6907,
 -1.898,  2.345,  1.275, -1.032,  1.275,   4.86,  1.003,  -1.24, 0.2059,  1.869, -2.309,  3.791,
      1, -1.879,  1.003,     -2, -1.879,  1.003,   3.97, -1.879,  1.003, 0.9703, -1.879,  1.003,
 -1.014,  1.281, 0.6657,  1.888,   1.85,  -1.24, -1.879,  3.187,  1.234,  1.022,  3.755,-0.6714,
 -1.898, -2.334,  3.774, -1.032,  -1.27, 0.2059,  1.003,  1.234,  4.839,  1.869,  2.299,   1.27,
     -2,  1.022,  1.869,      1,  1.022,  1.869, 0.9703,  1.022,  1.869,   3.97,  1.022,  1.869,
 -1.014,  0.206,   1.24,  1.888,  1.265, -2.309, -1.879,  3.755,  2.299,  1.022,  4.814,  -1.25,
 -1.898,   1.27,   1.85, -1.032, 0.6907,  3.791,  1.003,-0.6714,   1.27,  1.869,  -1.25,  3.212;

    testb.resize(12);
    testb<<
  -0.00981,
-1.458e-10,
 5.357e-10,
   -0.0098,
 -1.44e-10,
 5.298e-10,
 -0.009807,
-1.399e-10,
 5.375e-10,
 -0.009807,
-1.381e-10,
 5.316e-10;

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
