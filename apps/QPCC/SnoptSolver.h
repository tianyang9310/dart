#ifndef QPCC_SNOPT_H
#define QPCC_SNOPT_H

#include <vector>
#include <Eigen/Dense>
#include "SnoptInterface.h"
#include "Solver.h"

namespace qpcc {

class Problem;
class ConstraintBox;
class ObjectiveBox;
class Var;


class SnoptSolver : public Solver {
public:
    SnoptSolver(Problem *problem);
    virtual ~SnoptSolver();
    
    virtual bool solve();
    
    virtual void resetSolver();
    static int iterUpdate(long mask, int compute_gradients, double *coefs, void *update_data);
private:
    SnoptInterface *mSnopt;
    /* std::vector<Dofs> mVariables; */
    /* ConstrBox* mConstrBox; */
    /* ObjBox* mObjBox; */
    
    ConstraintBox* conBox();
    ObjectiveBox* objBox();
    
    bool mNoDisplay;
    int mSolverIter;
    
    int mOptCount;
    bool mPrint;
    int mUnit;
    
    
};

}
#endif // #ifndef OPTIMIZER_SNOPT_SNOPT_H

