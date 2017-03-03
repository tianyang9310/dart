#include <iostream>

#include "dart/dart.h"
#include "myLemke.h"
#include "csvParser.h"
// #include "dart/lcpsolver/Lemke.h"

int main(int argc, char* argv[]) 
{
    std::cout<<std::boolalpha<<std::setprecision(20);
    Eigen::MatrixXd testA;
    Eigen::VectorXd testb;
    Eigen::VectorXd* f;
    int err;

    f =  new Eigen::VectorXd(80);
    testA.resize(80,80);
    testA=load_csv<Eigen::MatrixXd>("/tmp/A.csv");
    // std::cout << testA << std::endl;

    testb.resize(80);
    testb=load_csv<Eigen::MatrixXd>("/tmp/b.csv");
    // std::cout << testb << std::endl;

    std::cout<<"Matrix A is"<<std::endl;
    std::cout<<testA<<std::endl;
    std::cout<<"Vector b is"<<std::endl;
    std::cout<<testb.transpose()<<std::endl;
    err = dart::lcpsolver::YT::Lemke(testA,testb,f);
    std::cout<<"Lemke solution z is"<<std::endl;
    std::cout<<(*f).transpose()<<std::endl;
    std::cout<<"Lemke error type is "<<err<<std::endl;

    std::cout<<std::endl<<std::endl;
    std::cout<<"-----Validation-----"<<std::endl;
    std::cout<<"z: "<<std::endl;
    std::cout<<(*f).transpose()<<std::endl;
    std::cout<<"A*z+b: "<<std::endl;
    std::cout<<(testA*(*f)+testb).transpose()<<std::endl;
    std::cout<<"LCP validation: "<<dart::lcpsolver::YT::validate(testA,testb,(*f))<<std::endl;

}
