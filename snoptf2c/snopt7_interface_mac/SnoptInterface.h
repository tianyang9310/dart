/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_SNOPT_SNOPT_INTERFACE_H
#define OPTIMIZER_SNOPT_SNOPT_INTERFACE_H

#include <vector>
#include <Eigen/Dense>
#include "optimizer/OptimizerArrayTypes.h"

#ifndef	ZERO
#define	ZERO 1.0e-30
#endif  // ifndef ZERO

namespace optimizer {
    namespace snopt {

        class SnoptInterface {
        public:
            enum Return { Solution, UserStop, Error, Stop, Infeasible};
            enum UpdateType { Obj = 1, Constr = 2 };
            enum SlackType { NoSlack = 0, Vslack = 1, Wslack = 2 };
            enum AbnormalType { NONE = 0, INFEASIBLE = 1, HESSIAN_UPDATE = 2, HESSIAN_RESET = 3 }; 

            struct Slack {
                long constr_idx;
                SlackType type;
                double val;
            };

            typedef int (*updateFunc) (long mask, int compute_gradients, double *coef_values, void *update_data);
		
            SnoptInterface(long constr_total, long coef_total, 
                           long nonlin_constr_total, long nonlin_obj_coef, long nonlin_jac_coef,
                           long *constr_eqns, long has_objective, VVD J, VVB JMap, std::vector<double> *constraints,
                           double *objective, std::vector<double> *gradient,	updateFunc update_f, void *update_d);
		
            SnoptInterface(long has_objective, VVD J, VVB JMap, std::vector<double> *constraints, double *objective,
                           std::vector<double> *gradient, updateFunc update_f, void *update_d);
            ~SnoptInterface();

            Return solve(double *x, double *lo_bounds, double *hi_bounds, long unit = 4);

            void clear(long mask, long compute_derivs);

            void resizeJacobian(long coef_total, long nonlin_coef_total, long constr_total, long nonlin_constr_total);
            void resizeCoef();

            long mNumConstr;
            long mNumCoef;
            long mNumNonlinConstr;
            long mNumNonlinObjCoef;
            long mNumNonlinJacCoef;

            double *mSolverX;
            double *mProblemX;
            double *mBoundsLo;
            double *mBoundsHi;
            long *mConstrEqns;

            long mHasObjective;

            static SnoptInterface *mRef;

            double *mObj;
            std::vector<double> *mdObjdCoef;

            std::vector<double> *mConstr;
            VVD mdConstrdCoef;

            Eigen::VectorXd mConstrScale;
            Eigen::VectorXd mCoefScale;

            VVB mCoefMap;
            double mReturnedObj;

            long mOutput;
            long mSum;
            bool mCheckTerm;
            bool mTermination;
            AbnormalType mAbnormal;
            long mBreak;

            void updateSolverX();
            void update(long mask, long compute_derivs, double *x);
            static void checkTermination(long *iAbort, double *xs);


        protected:
            SnoptInterface::updateFunc mUpdateFunc;
            void *mUpdateData;

            void scaleValues(long update_type, long compute_derivs);

        private:
            static void snoptObj(long *mode, long *nn_obj, double *x, 
                                 double *f_obj, double *g_obj, long *nstate, 
                                 char *cu, long *lencu, long *iu, long *leniu, 
                                 double *ru, long *lenru);
            static void snoptJac(long *mode, long *nn_con, long *nn_jac, long *ne_jac,
                                 double *x, double *f_con, double *g_con, long *nstate, 
                                 char *cu, long *lencu, long *iu, long *leniu, 
                                 double *ru, long *lenru);
            void fillUpSnoptFormat(VVD jacobian, double **a, long **ha, long **ka);
            long sparseCount(long col);

        };
        
    } // namespace snopt
} // namespace optimizer

#endif // #ifndef OPTIMIZER_SNOPT_SNOPT_INTERFACE_H

