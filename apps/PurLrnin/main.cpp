#include <iostream>
#include <vector>
#include <fstream>

#include "dart/dart.h"
//#include "MyWindow.h"
//#include "LPContactProblem.h"
//#include "SnoptSolver.h"
//#include "Var.h"

//using namespace contactLearning;

bool verify(size_t _n, double* _A, double* _x,
            double* lo, double* hi, double* b,
            double* w, int* findex, bool debug)
{
  size_t nSkip = dPAD(_n);
  if (debug) {
    std::cout << "A: " << std::endl;
    for (size_t i = 0; i < _n; ++i)
    {
      for (size_t j = 0; j < nSkip; ++j)
      {
        std::cout <<  _A[i * nSkip + j] << ", ";
      }
      std::cout << std::endl;
    }
    
    std::cout << "b: ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << b[i] << ", ";
    }
    std::cout << std::endl;
    
    std::cout << "w: ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << w[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "x: ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << _x[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "lb: ";
    for (int i = 0; i < _n; ++i)
    {
      std::cout << lo[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "ub: ";
    for (int i = 0; i < _n; ++i)
    {
      std::cout << hi[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "frictionIndex: ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << findex[i] << " ";
    }
    std::cout << std::endl;
  }
  
  double* Ax  = new double[_n];
  
  for (size_t i = 0; i < _n; ++i)
  {
    Ax[i] = 0.0;
  }
  
  for (size_t i = 0; i < _n; ++i)
  {
    for (size_t j = 0; j < _n; ++j)
    {
      Ax[i] += _A[i * nSkip + j] * _x[j];
    }
  }
  if(debug) {
    std::cout << "Ax   : ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << Ax[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "b + w: ";
    for (size_t i = 0; i < _n; ++i)
    {
      std::cout << b[i] + w[i] << " ";
    }
    std::cout << std::endl;
  }
  
  bool solved = true;
  for (int i = 0; i < _n; i++) {
    if (!dart::math::isZero(Ax[i] - (b[i] + w[i]))) {
      solved = false;
      break;
    }
  }
  delete[] Ax;
  return solved;
}

void recordLCPSolve(size_t _n, double* _A, double* _x, double* _w,
                    double* b, std::fstream &outputFile)
{
  size_t nSkip = dPAD(_n);
  
  for (size_t i = 0; i < _n; ++i)
  {
    for (size_t j = i; j < _n; ++j)
    {
      outputFile << _A[i * nSkip + j] << ", ";
    }
  }
  
  for (size_t i = 0; i < _n; ++i)
  {
    outputFile << b[i] << ", ";
  }
  
  int numContacts = _n / 3; // with friction
  for (size_t i = 0; i < numContacts; ++i)
  {
    int index = i * 3;
    double w_n = _w[index];
    double w_f1 = _w[index + 1];
    double w_f2 = _w[index + 2];
    
    int val = 0;
    if (w_n > 0)                            // x_n = lo = 0               ===> no normal force
      val = 0;
    else {                                  // x_n > lo = 0               ===> normal force
      if (w_f1 < 0 && w_f2 == 0)            // x_f1 = hi, lo < x_f2 < hi  ===> +f1
        val = 1;
      else if (w_f1 > 0 && w_f2 == 0)       // x_f1 = lo, lo < x_f2 < hi  ===> -f1
        val = 2;
      else if (w_f1 == 0 && w_f2 < 0)       // lo < x_f1 < hi, x_f2 = hi  ===> +f2
        val = 3;
      else if (w_f1 == 0 && w_f2 > 0)       // lo < x_f1 < hi, x_f2 = lo  ===> -f2
        val = 4;
      else if (w_f1 < 0 && w_f2 < 0)        // x_f1 = hi,      x_f2 = hi  ===> +f1 & +f2
        val = 5;
      else if (w_f1 > 0 && w_f2 < 0)        // x_f1 = lo,      x_f2 = hi  ===> -f1 & +f2
        val = 6;
      else if (w_f1 < 0 && w_f2 > 0)        // x_f1 = hi,      x_f2 = lo  ===> +f1 & -f2
        val = 7;
      else if (w_f1 > 0 && w_f2 > 0)        // x_f1 = lo,      x_f2 = lo  ===> -f1 & -f2
        val = 8;
      else                                  // lo<x_f1<hi,  lo<x_f2<hi    ===> 0 no friction force
        val = 9;
    }
    if (i == numContacts - 1)
      outputFile << val;
    else
      outputFile << val << ", ";
  }
  outputFile << std::endl;
}



int main(int argc, char* argv[]) {

  int numContactsToLearn = 4;
  int n = numContactsToLearn * 3; // with friction
  std::string trainingFile =
  "/tmp/lcp_data" + std::to_string(numContactsToLearn) + ".cvs";
  
  std::fstream outputFile(trainingFile, std::fstream::out);
  outputFile.precision(10);

  int nSkip = dPAD(n);
  double* A = new double[n * nSkip];
  double* oldA = new double[n * nSkip];
  double* x = new double[n];
  double* b = new double[n];
  double* oldb = new double[n];
  double* w = new double[n];
  double* lo = new double[n];
  double* hi = new double[n];
  int* findex = new int[n];
  
  double minA[] = {
  0, -9,-10,  0, -9,-10,-10, -9,-10,  0, -9,-10,
 -9,  0, -7, -9,  0,-10,-10, -1, -6,-10,  0,-14,
-10, -7,  0,-10,-14,  0, -9, -6, -2,-10,-13,  0,
  0, -9,-10,  0, -9,-10,  0, -9,-10,-10, -9,-10,
 -9,  0,-14, -9,  0, -6,-10,  0,-13,-10, -2, -7,
-10,-10,  0,-10, -6,  0, -9,-14,  0,-10, -7, -1,
-10,-10, -9,  0,-10, -9,  0,-10, -9,  0,-10, -9,
 -9, -1, -6, -9,  0,-14,-10,  0, -7,-10,  0,-10,
-10, -6, -2,-10,-13,  0, -9, -7,  0,-10,-15,  0,
  0,-10,-10,-10,-10,-10,  0,-10,-10,  0,-10,-10,
 -9,  0,-13, -9, -2, -7,-10,  0,-15,-10,  0, -6,
-10,-14,  0,-10, -7, -1, -9,-10,  0,-10, -6,  0};
    
  double maxA[] = {
21,11,10, 6,11,10, 1,11,10, 6,11,10,
11,27, 7,11,20,15,10,13, 8,10,20,11,
10, 7,28,11,14,20,11, 8,12,11,15,20,
 6,11,11,21,11,11, 6,11,11, 1,11,11,
11,20,14,11,28, 8,10,20,15,10,12, 7,
10,15,20,11, 8,27,11,11,20,11, 7,13,
 1,10,11, 6,10,11,21,10,11, 6,10,11,
11,13, 8,11,20,11,10,27, 7,10,20,15,
10, 8,12,11,15,20,11, 7,28,11,14,20,
 6,10,11, 1,10,11, 6,10,11,21,10,11,
11,20,15,11,12, 7,10,20,14,10,28, 8,
10,11,20,11, 7,13,11,15,20,11, 8,27};


  // -----------------------------------------------
  // debug minA and maxA
  Eigen::Map<Eigen::MatrixXd> Eigen_minA(minA,12,12);
  Eigen::Map<Eigen::MatrixXd> Eigen_maxA(maxA,12,12);
  std::cout<<Eigen_minA<<std::endl;
  std::cout<<Eigen_maxA<<std::endl;
  // -----------------------------------------------

  double minB[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double maxB[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  dart::math::seedRand();
  int sampleCount = 0;
  while (sampleCount < 30000) {
    for (int i = 0; i < n; i++) {
      for (int j = i; j < n; j++) {
        A[i * nSkip + j] =
                  dart::math::random(minA[i * n + j],
                             maxA[i * n + j]);
        A[j * nSkip + i] = A[i * nSkip + j];
      }
    }
    for (int i = 0; i < n; i++)
      b[i] = dart::math::random(minB[i], maxB[i]);
    
    for (int i = 0; i < n * nSkip; i++)
      oldA[i] = A[i];
    for (int i = 0; i < n; i++)
      oldb[i] = b[i];
    
    for (int i = 0; i < numContactsToLearn; i++) {
      int index = i * 3;
      lo[index] = 0.0;
      hi[index] = dInfinity;
      w[index] = 0.0;
      x[index] = 0.0;
      findex[index] = -1;
      
      lo[index + 1] = -1.0;
      hi[index + 1] = 1.0;
      w[index + 1] = 0.0;
      x[index + 1] = 0.0;
      findex[index + 1] = index;

      lo[index + 2] = -1.0;
      hi[index + 2] = 1.0;
      w[index + 2] = 0.0;
      x[index + 2] = 0.0;
      findex[index + 2] = index;
    }
    
    dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);
    bool solved = verify(n, oldA, x, lo, hi, oldb, w, findex, false);
    if (solved) {
      recordLCPSolve(n, oldA, x, w, oldb, outputFile);
      sampleCount++;
      std::cout << sampleCount << std::endl;
    } else {
      std::cout << "not solved!" << std::endl;
    }
  }
  
  outputFile.close();
  
  delete[] A;
  delete[] oldA;
  delete[] x;
  delete[] b;
  delete[] oldb;
  delete[] w;
  delete[] lo;
  delete[] hi;
  delete[] findex;
  return 0;
}

