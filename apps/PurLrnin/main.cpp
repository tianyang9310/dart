#include <iostream>
#include <vector>
#include <fstream>

#include "apps/lemkeFix/myLemke.h"
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


void recordLCPSolve(const Eigen::MatrixXd A,
                    const Eigen::VectorXd z,
                    const Eigen::VectorXd b, 
                    std::fstream &outputFile) {
  int numBasis = 8;
  int nSize = b.rows();
  int numContactsToLearn = nSize / (numBasis + 2);
  assert(numContactsToLearn == numContactsCallBack);

  //  output A, z, and b
  //  // since all friction coeffs are the same, no need to output them
  for (int i = 0; i < nSize - numContactsToLearn; i++) {
    for (int j = i; j < nSize - numContactsToLearn; j++) {
      outputFile << A(i, j) << ",";
    }
  }

  for (int i = 0; i < numContactsToLearn; i++) {
    outputFile << A(nSize-numContactsToLearn+i,i) << ",";
  }

  for (int i = 0; i < nSize - numContactsToLearn; i++) {
    outputFile << b(i) << ",";
  }

  // decompose z
  Eigen::VectorXd z_fn(numContactsToLearn);
  z_fn = z.head(numContactsToLearn);
  Eigen::VectorXd z_fd(numContactsToLearn * numBasis);
  z_fd = z.segment(numContactsToLearn, numContactsToLearn * numBasis);
  Eigen::VectorXd z_lambda(numContactsToLearn);
  z_lambda = z.tail(numContactsToLearn);

  double RECORD_ZERO = 1e-25;
  int value = 9;
  std::vector<Eigen::VectorXd> each_z(numContactsToLearn);
  for (int i = 0; i < numContactsToLearn; i++) {
    each_z[i].resize(numBasis + 2);
    each_z[i] << z_fn(i), z_fd.segment(i * numBasis, numBasis), z_lambda(i);
    
    int value = 9;
    // Convention: numbasis = 8, so total 10 elements
    if (each_z[i](0) < RECORD_ZERO)  // fn = 0, break
    {
      value = 9;
    } else if (each_z[i](numBasis+1) < RECORD_ZERO) { // lambda = 0, static
      value = 8;
    } else { // random choose non-zero in fd
      std::vector<int> nonZerofd;
      nonZerofd.clear();
      for (int j = 0; j < numBasis; j++) {
        if (each_z[i](j+1) > RECORD_ZERO) {
          nonZerofd.push_back(j);
        }
      }
      assert(nonZerofd.size() <= 2);
      assert(nonZerofd.size() > 0);
      value = nonZerofd[std::rand() % nonZerofd.size()];
    }
    outputFile << value;
    if (i < numContactsToLearn - 1) {
      outputFile << ",";
    }
  }

  outputFile << std::endl;
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

  int numBasis = 8;
  int numContactsToLearn = 2;
  int Arow = numContactsToLearn * (numBasis + 2);
  int Acol = Arow;
  int ACompactRow = numContactsToLearn * (numBasis + 1); 
  int ACompactCol = ACompactRow;
  
  std::string trainingFile =
  "/tmp/CntctLrnin/synthesis_lcp_data" + std::to_string(numContactsToLearn) + ".csv";
  std::fstream outputFile(trainingFile, std::fstream::out);
  outputFile.precision(20);

  // int nSkip = dPAD(n);
  // double* A = new double[n * nSkip];
  // double* oldA = new double[n * nSkip];
  // double* x = new double[n];
  // double* b = new double[n];
  // double* oldb = new double[n];
  // double* w = new double[n];
  // double* lo = new double[n];
  // double* hi = new double[n];
  // int* findex = new int[n];

double maxA[] = {
3.9995,0.99882,2.2495,2.2499,2.2497,2.247,2.2495,2.249,2.2495,2.2496,2.2903,2.2529,2.2623,2.2597,2.2686,2.2763,2.273,2.2747,
0.99882,3.9993,2.2793,2.2572,2.2629,2.2681,2.2848,2.2711,2.2827,2.2672,2.2486,2.2472,2.2493,2.2497,2.2496,2.2494,2.2496,2.2496,
2.2495,2.2793,5.5,4.3265,1.4976,-1.3384,-2.5129,-1.3357,1.4945,4.3275,4,4.3223,2.9955,0.78099,-1.0101,0.77679,2.9869,4.3221,
2.2499,2.2572,4.3265,5.5,4.3251,1.4918,-1.3357,-2.5099,-1.332,1.4968,4.3164,4,4.3215,2.9835,0.77901,-1.0118,0.78174,2.9944,
2.2497,2.2629,1.4976,4.3251,5.5,4.3269,1.4945,-1.332,-2.5057,-1.3332,2.9824,4.3176,4,4.3205,2.9821,0.778,-1.0055,0.77066,
2.247,2.2681,-1.3384,1.4918,4.3269,5.5,4.3275,1.4968,-1.3332,-2.5044,0.77255,2.9899,4.3213,4,4.322,2.9908,0.77946,-1.0045,
2.2495,2.2848,-2.5129,-1.3357,1.4945,4.3275,5.5,4.3265,1.4976,-1.3384,-1.0101,0.77679,2.9869,4.3221,4,4.3223,2.9955,0.78099,
2.249,2.2711,-1.3357,-2.5099,-1.332,1.4968,4.3265,5.5,4.3251,1.4918,0.77901,-1.0118,0.78174,2.9944,4.3164,4,4.3215,2.9835,
2.2495,2.2827,1.4945,-1.332,-2.5057,-1.3332,1.4976,4.3251,5.5,4.3269,2.9821,0.778,-1.0055,0.77066,2.9824,4.3176,4,4.3205,
2.2496,2.2672,4.3275,1.4968,-1.3332,-2.5044,-1.3384,1.4918,4.3269,5.5,4.322,2.9908,0.77946,-1.0045,0.77255,2.9899,4.3213,4,
2.2903,2.2486,4,4.3164,2.9824,0.77255,-1.0101,0.77901,2.9821,4.322,5.5,4.326,1.4959,-1.3389,-2.5032,-1.3354,1.4965,4.3257,
2.2529,2.2472,4.3223,4,4.3176,2.9899,0.77679,-1.0118,0.778,2.9908,4.326,5.5,4.3263,1.4981,-1.3354,-2.5067,-1.3385,1.4919,
2.2623,2.2493,2.9955,4.3215,4,4.3213,2.9869,0.78174,-1.0055,0.77946,1.4959,4.3263,5.5,4.3256,1.4965,-1.3385,-2.5133,-1.3388,
2.2597,2.2497,0.78099,2.9835,4.3205,4,4.3221,2.9944,0.77066,-1.0045,-1.3389,1.4981,4.3256,5.5,4.3257,1.4919,-1.3388,-2.5063,
2.2686,2.2496,-1.0101,0.77901,2.9821,4.322,4,4.3164,2.9824,0.77255,-2.5032,-1.3354,1.4965,4.3257,5.5,4.326,1.4959,-1.3389,
2.2763,2.2494,0.77679,-1.0118,0.778,2.9908,4.3223,4,4.3176,2.9899,-1.3354,-2.5067,-1.3385,1.4919,4.326,5.5,4.3263,1.4981,
2.273,2.2496,2.9869,0.78174,-1.0055,0.77946,2.9955,4.3215,4,4.3213,1.4965,-1.3385,-2.5133,-1.3388,1.4959,4.3263,5.5,4.3256,
2.2747,2.2496,4.3221,2.9944,0.77066,-1.0045,0.78099,2.9835,4.3205,4,4.3257,1.4919,-1.3388,-2.5063,-1.3389,1.4981,4.3256,5.5};
double minA[] = {
2.4994,-0.5,-2.2497,-2.2492,-2.2496,-2.2498,-2.2495,-2.2499,-2.2498,-2.2473,-2.2718,-2.2765,-2.2731,-2.2753,-2.2924,-2.2569,-2.2625,-2.2604,
-0.5,2.4828,-2.2852,-2.2721,-2.2828,-2.2677,-2.2794,-2.2579,-2.2642,-2.2685,-2.2497,-2.2495,-2.2496,-2.2497,-2.249,-2.2473,-2.2494,-2.2497,
-2.2497,-2.2852,2.5126,1.3346,-1.4945,-4.3279,-5.5,-4.3265,-1.4977,1.3381,1.0095,-0.78263,-2.9882,-4.3222,-4,-4.3231,-2.9956,-0.78132,
-2.2492,-2.2721,1.3346,2.5098,1.3313,-1.497,-4.3265,-5.5,-4.3252,-1.4919,-0.78055,1.0106,-0.78228,-2.9945,-4.3171,-4,-4.3219,-2.9839,
-2.2496,-2.2828,-1.4945,1.3313,2.5056,1.3329,-1.4977,-4.3252,-5.5,-4.3269,-2.9831,-0.77922,1.0049,-0.77195,-2.9834,-4.3181,-4,-4.321,
-2.2498,-2.2677,-4.3279,-1.497,1.3329,2.5042,1.3381,-1.4919,-4.3269,-5.5,-4.3221,-2.991,-0.78083,1.0044,-0.77339,-2.9905,-4.3241,-4,
-2.2495,-2.2794,-5.5,-4.3265,-1.4977,1.3381,2.5126,1.3346,-1.4945,-4.3279,-4,-4.3231,-2.9956,-0.78132,1.0095,-0.78263,-2.9882,-4.3222,
-2.2499,-2.2579,-4.3265,-5.5,-4.3252,-1.4919,1.3346,2.5098,1.3313,-1.497,-4.3171,-4,-4.3219,-2.9839,-0.78055,1.0106,-0.78228,-2.9945,
-2.2498,-2.2642,-1.4977,-4.3252,-5.5,-4.3269,-1.4945,1.3313,2.5056,1.3329,-2.9834,-4.3181,-4,-4.321,-2.9831,-0.77922,1.0049,-0.77195,
-2.2473,-2.2685,1.3381,-1.4919,-4.3269,-5.5,-4.3279,-1.497,1.3329,2.5042,-0.77339,-2.9905,-4.3241,-4,-4.3221,-2.991,-0.78083,1.0044,
-2.2718,-2.2497,1.0095,-0.78055,-2.9831,-4.3221,-4,-4.3171,-2.9834,-0.77339,2.5024,1.3353,-1.4965,-4.3257,-5.5,-4.3261,-1.4959,1.338,
-2.2765,-2.2495,-0.78263,1.0106,-0.77922,-2.991,-4.3231,-4,-4.3181,-2.9905,1.3353,2.5067,1.3376,-1.4921,-4.3261,-5.5,-4.3264,-1.4987,
-2.2731,-2.2496,-2.9882,-0.78228,1.0049,-0.78083,-2.9956,-4.3219,-4,-4.3241,-1.4965,1.3376,2.5112,1.3387,-1.4959,-4.3264,-5.5,-4.3258,
-2.2753,-2.2497,-4.3222,-2.9945,-0.77195,1.0044,-0.78132,-2.9839,-4.321,-4,-4.3257,-1.4921,1.3387,2.506,1.338,-1.4987,-4.3258,-5.5,
-2.2924,-2.249,-4,-4.3171,-2.9834,-0.77339,1.0095,-0.78055,-2.9831,-4.3221,-5.5,-4.3261,-1.4959,1.338,2.5024,1.3353,-1.4965,-4.3257,
-2.2569,-2.2473,-4.3231,-4,-4.3181,-2.9905,-0.78263,1.0106,-0.77922,-2.991,-4.3261,-5.5,-4.3264,-1.4987,1.3353,2.5067,1.3376,-1.4921,
-2.2625,-2.2494,-2.9956,-4.3219,-4,-4.3241,-2.9882,-0.78228,1.0049,-0.78083,-1.4959,-4.3264,-5.5,-4.3258,-1.4965,1.3376,2.5112,1.3387,
-2.2604,-2.2497,-0.78132,-2.9839,-4.321,-4,-4.3222,-2.9945,-0.77195,1.0044,1.338,-1.4987,-4.3258,-5.5,-4.3257,-1.4921,1.3387,2.506};
double maxb[] = {
0.075659,0.080003,1.4461,1.319,1.2741,1.3034,1.6114,2.1396,1.7646,1.6805,1.4725,1.4397,1.3395,1.2475,1.5017,1.7173,1.8007,2.0055};
double minb[] = {
-2.5785,-2.4742,-1.7661,-2.6762,-1.7686,-1.8117,-1.4486,-1.3214,-1.5821,-1.337,-1.6122,-2.0232,-1.8307,-2.0869,-1.6217,-1.752,-1.3853,-1.3129};

  // -----------------------------------------------
  // debug minA and maxA
  Eigen::Map<Eigen::MatrixXd> Eigen_minA(minA,18,18);
  Eigen::Map<Eigen::MatrixXd> Eigen_maxA(maxA,18,18);
  std::cout<<Eigen_minA<<std::endl;
  std::cout<<Eigen_maxA<<std::endl;
  // -----------------------------------------------

  dart::math::seedRand();
  int sampleCount = 0;
  Eigen::MatrixXd A(Arow, Acol);
  Eigen::VectorXd b(Arow);
  Eigen::VectorXd* z = new Eigen::VectorXd(numContactsToLearn * (2 + numBasis));
  A.setZero();
  b.setZero();
  z->setZero();
  while (sampleCount < 30000) {
    for (int i = 0; i < ACompactRow; i++) {
      for (int j = 0; j < ACompactCol; j++) {
        A(i,j) = dart::math::random(Eigen_minA(i,j), Eigen_maxA(i,j));
      }
      b(i) = dart::math::random(minb[i],maxb[i]);
    }
    
    int err = dart::lcpsolver::YT::Lemke(A, b, z);

    double err_dist = 0.0;
    bool Validation =
        dart::lcpsolver::YT::validate(A, (*z), b, err_dist);

    if (Validation) {
      recordLCPSolve(A,(*z),b,outputFile);
      sampleCount++;
      std::cout << sampleCount << std::endl;
    } else {
      std::cout << "not solved!" << std::endl;
    }
  }
  
  outputFile.close();
  
  return 0;
}

