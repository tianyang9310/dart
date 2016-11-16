#include <iostream>

#include "dart/dart.h"
#include "MyLemke.h"
// #include "dart/lcpsolver/Lemke.h"

int main(int argc, char* argv[]) 
{
    std::cout<<std::boolalpha;
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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;

    std::cout<<"=========================================="<<std::endl;
    f =  new Eigen::VectorXd(20);
    testA.resize(20,20);
    testA <<
2.4929,-0.49884,-2.0132,-0.95655,0.66045,1.8906,2.0132,0.95655,-0.66045,-1.8906,-2.0077,-0.95391,0.65863,1.8854,2.0077,0.95391,-0.65863,-1.8854,0,0,
-0.49884,2.5094,1.9847,0.85554,-0.77474,-1.9512,-1.9847,-0.85554,0.77474,1.9512,1.9792,0.85318,-0.7726,-1.9458,-1.9792,-0.85318,0.7726,1.9458,0,0,
-2.0132,1.9847,4.1522,3.2487,0.44216,-2.6234,-4.1522,-3.2487,-0.44216,2.6234,3.8287,2.3991,-0.43588,-3.0155,-3.8287,-2.3991,0.43588,3.0155,1,0,
-0.95655,0.85554,3.2487,5.1957,4.0992,0.60138,-3.2487,-5.1957,-4.0992,-0.60138,2.3405,2.7721,1.5799,-0.53787,-2.3405,-2.7721,-1.5799,0.53787,1,0,
0.66045,-0.77474,0.44216,4.0992,5.3549,3.4739,-0.44216,-4.0992,-5.3549,-3.4739,-0.51867,1.5213,2.6702,2.2548,0.51867,-1.5213,-2.6702,-2.2548,1,0,
1.8906,-1.9512,-2.6234,0.60138,3.4739,4.3114,2.6234,-0.60138,-3.4739,-4.3114,-3.0741,-0.62066,2.1963,3.7267,3.0741,0.62066,-2.1963,-3.7267,1,0,
2.0132,-1.9847,-4.1522,-3.2487,-0.44216,2.6234,4.1522,3.2487,0.44216,-2.6234,-3.8287,-2.3991,0.43588,3.0155,3.8287,2.3991,-0.43588,-3.0155,1,0,
0.95655,-0.85554,-3.2487,-5.1957,-4.0992,-0.60138,3.2487,5.1957,4.0992,0.60138,-2.3405,-2.7721,-1.5799,0.53787,2.3405,2.7721,1.5799,-0.53787,1,0,
-0.66045,0.77474,-0.44216,-4.0992,-5.3549,-3.4739,0.44216,4.0992,5.3549,3.4739,0.51867,-1.5213,-2.6702,-2.2548,-0.51867,1.5213,2.6702,2.2548,1,0,
-1.8906,1.9512,2.6234,-0.60138,-3.4739,-4.3114,-2.6234,0.60138,3.4739,4.3114,3.0741,0.62066,-2.1963,-3.7267,-3.0741,-0.62066,2.1963,3.7267,1,0,
-2.0077,1.9792,3.8287,2.3405,-0.51867,-3.0741,-3.8287,-2.3405,0.51867,3.0741,4.1902,3.3244,0.51131,-2.6013,-4.1902,-3.3244,-0.51131,2.6013,0,1,
-0.95391,0.85318,2.3991,2.7721,1.5213,-0.62066,-2.3991,-2.7721,-1.5213,0.62066,3.3244,5.2566,4.1095,0.55512,-3.3244,-5.2566,-4.1095,-0.55512,0,1,
0.65863,-0.7726,-0.43588,1.5799,2.6702,2.1963,0.43588,-1.5799,-2.6702,-2.1963,0.51131,4.1095,5.3004,3.3864,-0.51131,-4.1095,-5.3004,-3.3864,0,1,
1.8854,-1.9458,-3.0155,-0.53787,2.2548,3.7267,3.0155,0.53787,-2.2548,-3.7267,-2.6013,0.55512,3.3864,4.234,2.6013,-0.55512,-3.3864,-4.234,0,1,
2.0077,-1.9792,-3.8287,-2.3405,0.51867,3.0741,3.8287,2.3405,-0.51867,-3.0741,-4.1902,-3.3244,-0.51131,2.6013,4.1902,3.3244,0.51131,-2.6013,0,1,
0.95391,-0.85318,-2.3991,-2.7721,-1.5213,0.62066,2.3991,2.7721,1.5213,-0.62066,-3.3244,-5.2566,-4.1095,-0.55512,3.3244,5.2566,4.1095,0.55512,0,1,
-0.65863,0.7726,0.43588,-1.5799,-2.6702,-2.1963,-0.43588,1.5799,2.6702,2.1963,-0.51131,-4.1095,-5.3004,-3.3864,0.51131,4.1095,5.3004,3.3864,0,1,
-1.8854,1.9458,3.0155,0.53787,-2.2548,-3.7267,-3.0155,-0.53787,2.2548,3.7267,2.6013,-0.55512,-3.3864,-4.234,-2.6013,0.55512,3.3864,4.234,0,1,
1,0,-1,-1,-1,-1,-1,-1,-1,-1,-0,-0,-0,-0,-0,-0,-0,-0,0,0,
0,1,-0,-0,-0,-0,-0,-0,-0,-0,-1,-1,-1,-1,-1,-1,-1,-1,0,0;

    testb.resize(20);
    testb<<
0.014552,
-1.1572,
0.11697,
0.055577,
-0.038373,
-0.10984,
-0.11697,
-0.055577,
0.038373,
0.10984,
0.11915,
0.056611,
-0.039087,
-0.11189,
-0.11915,
-0.056611,
0.039087,
0.11189,
0,
0;

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
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,(*f),testb)<<std::endl;
}
