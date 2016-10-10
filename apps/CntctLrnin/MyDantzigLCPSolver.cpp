#include "MyDantzigLCPSolver.h"

MyDantzigLCPSolver::MyDantzigLCPSolver(double _timestep,int _totalDOF):DantzigLCPSolver(_timestep),totalDOF(_totalDOF)
{
    
}

void MyDantzigLCPSolver::solve(ConstrainedGroup* _group)
{
    // If there is no constraint, then just return true.
    size_t numConstraints = _group->getNumConstraints(); // numConstraints is exactly the number of contact points 
    if (numConstraints == 0)
        return;
    
    // Build LCP terms by aggregating them from constraints
    size_t n = _group->getTotalDimension(); // n = 3*numConstraints if mIsFrictionOn else numConstraints
    int nSkip = dPAD(n);
    double* A = new double[n * nSkip];
    double* x = new double[n];
    double* b = new double[n];
    double* w = new double[n];
    double* lo = new double[n];
    double* hi = new double[n];
    int* findex = new int[n];
    
    // Set w to 0 and findex to -1
#ifndef NDEBUG
    std::memset(A, 0.0, n * nSkip * sizeof(double));
#endif
    std::memset(w, 0.0, n * sizeof(double));
    std::memset(findex, -1, n * sizeof(int));
    
    // Compute offset indices
    size_t* offset = new size_t[n];
    offset[0] = 0;
    //  std::cout << "offset[" << 0 << "]: " << offset[0] << std::endl;
    for (size_t i = 1; i < numConstraints; ++i)
    {
        ConstraintBase* constraint = _group->getConstraint(i - 1);
        assert(constraint->getDimension() > 0);
        offset[i] = offset[i - 1] + constraint->getDimension();
        //    std::cout << "offset[" << i << "]: " << offset[i] << std::endl;
    }
    
    // For each constraint
    ConstraintInfo constInfo;
    constInfo.invTimeStep = 1.0 / mTimeStep;
    ConstraintBase* constraint;
    for (size_t i = 0; i < numConstraints; ++i)
    {
        constraint = _group->getConstraint(i);
        
        constInfo.x      = x      + offset[i];
        constInfo.lo     = lo     + offset[i];
        constInfo.hi     = hi     + offset[i];
        constInfo.b      = b      + offset[i];
        constInfo.findex = findex + offset[i];
        constInfo.w      = w      + offset[i];
        
        // Fill vectors: lo, hi, b, w
        constraint->getInformation(&constInfo);
        
        // Fill a matrix by impulse tests: A
        constraint->excite();
        for (size_t j = 0; j < constraint->getDimension(); ++j)
        {
            // Adjust findex for global index
            if (findex[offset[i] + j] >= 0)
            {
                // std::cout<<"Adject findex for global index"<<std::endl;
                findex[offset[i] + j] += offset[i];
                // std::cin.get();
            }
            
            // Apply impulse for mipulse test
            constraint->applyUnitImpulse(j);
            
            // Fill upper triangle blocks of A matrix
            int index = nSkip * (offset[i] + j) + offset[i];
            constraint->getVelocityChange(A + index, true);
            for (size_t k = i + 1; k < numConstraints; ++k)
            {
                index = nSkip * (offset[i] + j) + offset[k];
                _group->getConstraint(k)->getVelocityChange(A + index, false);
            }
            
            // Filling symmetric part of A matrix
            for (size_t k = 0; k < i; ++k)
            {
                for (size_t l = 0; l < _group->getConstraint(k)->getDimension(); ++l)
                {
                    int index1 = nSkip * (offset[i] + j) + offset[k] + l;
                    int index2 = nSkip * (offset[k] + l) + offset[i] + j;
                    
                    A[index1] = A[index2];
                }
            }
        }
        
        assert(isSymmetric(n, A, offset[i],
                           offset[i] + constraint->getDimension() - 1));
        
        constraint->unexcite();
    }
    
    assert(isSymmetric(n, A));
    
    // Print LCP formulation
    // dtdbg << "Before solve:" << std::endl;
    // print(n, A, x, lo, hi, b, w, findex);
    // std::cout << std::endl;

//  ---------------------------------------
    // establish A and b matrix for Lemke algorithm
    ContactConstraint* cntctconstraint; 
    Eigen::MatrixXd M(totalDOF,totalDOF); 
    M.setIdentity();
    Eigen::MatrixXd N(totalDOF,numConstraints); 
    N.setZero();
    for (size_t idx_cnstrnt = 0; idx_cnstrnt < numConstraints; idx_cnstrnt++)
    {
        cntctconstraint = dynamic_cast<ContactConstraint*>(_group->getConstraint(idx_cnstrnt));
        // std::cout<<cntctconstraint->mContacts.size()<<std::endl;
        if (cntctconstraint->mContacts.size() > 1)
        {
            dtmsg<<"ERROR: more than one contact for current ContactConstraint"<<std::endl;
            std::cin.get();
        }
        dart::collision::Contact* ct = cntctconstraint->mContacts[0];
        Eigen::Vector3d bodyPoint1;
        Eigen::Vector3d bodyPoint2;
        bodyPoint1.noalias() = cntctconstraint->mBodyNode1->getTransform().inverse() * ct->point;
        bodyPoint2.noalias() = cntctconstraint->mBodyNode2->getTransform().inverse() * ct->point;

        dart::math::LinearJacobian J1;  // J1 and BodyNode1 is 
        dart::math::LinearJacobian J2;
        J1 = cntctconstraint->mBodyNode1->getLinearJacobian(bodyPoint1);
        J2 = cntctconstraint->mBodyNode2->getLinearJacobian(bodyPoint2);
        assert(J1.rows()=J2.rows());

        Eigen::Vector3d normDirection;
        normDirection = ct->normal;

        N.col(idx_cnstrnt) << J1.transpose()*normDirection,-J2.transpose()*normDirection;

        // TODO: here mass matrix should be computed beforehand
        Eigen::MatrixXd M1;
        Eigen::MatrixXd M2;
        M1 = cntctconstraint->mBodyNode1->getSkeleton()->getMassMatrix();
        M2 = cntctconstraint->mBodyNode2->getSkeleton()->getMassMatrix();
        
        M.block(0,0,M1.rows(),M1.cols()) = M1;
        M.block(M1.rows(),M1.cols(),M2.rows(),M2.cols()) = M2;
    }
    Eigen::MatrixXd __Lemke__A;
    __Lemke__A = N.transpose()*M.ldlt().solve(N);
    std::cout<<__Lemke__A<<std::endl;
    std::cin.get();
    
//  ---------------------------------------
    
/*
//  ---------------------------------------
    // Borrow A and b for Lemke
    std::cout<<"-----------------------------"<<std::endl;
    std::cout<<"---Solve LCP via Lemke-------"<<std::endl;
    if (!isSymmetric(n, A))
    // if (n != nSkip)
    {
        std::cout<<nSkip<<std::endl;
        std::cout<<n<<std::endl;
        std::cin.get();
    }
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> _eigen_A(A,n,nSkip);
    // cropping _eigen_A to be square matrix
    Eigen::MatrixXd _eigen_A_cropped(n,n);
    _eigen_A_cropped = _eigen_A.block(0,0,n,n);
    
    std::cout<<"Eigen::MatrixXd A is now"<<std::endl;
    std::cout<<_eigen_A_cropped<<std::endl;
    Eigen::Map<Eigen::VectorXd> _eigen_b(b,n);
    // negating b to be true LCP
    Eigen::VectorXd _eigen_b_neg(n);
    _eigen_b_neg = - _eigen_b;
    
    std::cout<<"Eigen::VectorXd b is now"<<std::endl;
    std::cout<<_eigen_b_neg<<std::endl;

    // Solve LCP using Lemke's algorithm
    // A and b are reference
    // z is pointer
    Eigen::VectorXd* z = new Eigen::VectorXd(n);
    int err = dart::lcpsolver::Lemke(_eigen_A_cropped,_eigen_b_neg,z);
    
    // Print LCP formulation
    std::cout<<"Using Lemke to solve the LCP problem"<<std::endl;
    std::cout<<"error: "<<err<<std::endl;
    std::cout<<"z "<<(*z).transpose()<<std::endl;
    std::cout<<"-----------------------------"<<std::endl;
    if (!dart::lcpsolver::validate(_eigen_A_cropped, *z, _eigen_b_neg))
    {
        std::cout<<"invalid Lemke solution"<<std::endl;
        // std::cin.get();
    }
//  ---------------------------------------
*/

    // Solve LCP using ODE's Dantzig algorithm
    dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);
    
    // Print LCP formulation
    // dtdbg << "After solve:" << std::endl;
    // print(n, A, x, lo, hi, b, w, findex);
    // std::cout << std::endl;
    
    // Apply constraint impulses
    for (size_t i = 0; i < numConstraints; ++i)
    {
        constraint = _group->getConstraint(i);
        constraint->applyImpulse(x + offset[i]);
        constraint->excite();
    }
    
    delete[] offset;
    
    delete[] A;
    delete[] x;
    delete[] b;
    delete[] w;
    delete[] lo;
    delete[] hi;
    delete[] findex;
}

void MyDantzigLCPSolver::print(size_t _n, double* _A, double* _x,
                               double* lo, double* hi, double* b,
                               double* w, int* findex)
{
    size_t nSkip = dPAD(_n);
    std::cout << "A: " << std::endl;
    for (size_t i = 0; i < _n; ++i)
    {
        for (size_t j = 0; j < nSkip; ++j)
        {
            std::cout << std::setprecision(4) << _A[i * nSkip + j] << " ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "b: ";
    for (size_t i = 0; i < _n; ++i)
    {
        std::cout << std::setprecision(4) << b[i] << " ";
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
    
    //  std::cout << "lb: ";
    //  for (int i = 0; i < dim; ++i)
    //  {
    //    std::cout << lb[i] << " ";
    //  }
    //  std::cout << std::endl;
    
    //  std::cout << "ub: ";
    //  for (int i = 0; i < dim; ++i)
    //  {
    //    std::cout << ub[i] << " ";
    //  }
    //  std::cout << std::endl;
    
    std::cout << "frictionIndex: ";
    for (size_t i = 0; i < _n; ++i)
    {
        std::cout << findex[i] << " ";
    }
    std::cout << std::endl;
    
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
    
    delete[] Ax;
}

bool MyDantzigLCPSolver::isSymmetric(size_t _n, double* _A)
{
    size_t nSkip = dPAD(_n);
    for (size_t i = 0; i < _n; ++i)
    {
        for (size_t j = 0; j < _n; ++j)
        {
            if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6)
            {
                std::cout << "A: " << std::endl;
                for (size_t k = 0; k < _n; ++k)
                {
                    for (size_t l = 0; l < nSkip; ++l)
                    {
                        std::cout << std::setprecision(4) << _A[k * nSkip + l] << " ";
                    }
                    std::cout << std::endl;
                }
                
                std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j] << std::endl;
                std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i] << std::endl;
                return false;
            }
        }
    }
    
    return true;
}

//==============================================================================
bool MyDantzigLCPSolver::isSymmetric(size_t _n, double* _A,
                                   size_t _begin, size_t _end)
{
    size_t nSkip = dPAD(_n);
    for (size_t i = _begin; i <= _end; ++i)
    {
        for (size_t j = _begin; j <= _end; ++j)
        {
            if (std::abs(_A[nSkip * i + j] - _A[nSkip * j + i]) > 1e-6)
            {
                std::cout << "A: " << std::endl;
                for (size_t k = 0; k < _n; ++k)
                {
                    for (size_t l = 0; l < nSkip; ++l)
                    {
                        std::cout << std::setprecision(4) << _A[k * nSkip + l] << " ";
                    }
                    std::cout << std::endl;
                }
                
                std::cout << "A(" << i << ", " << j << "): " << _A[nSkip * i + j] << std::endl;
                std::cout << "A(" << j << ", " << i << "): " << _A[nSkip * j + i] << std::endl;
                return false;
            }
        }
    }
    
    return true;
}

