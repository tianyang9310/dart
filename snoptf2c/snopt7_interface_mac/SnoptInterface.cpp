/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "SnoptInterface.h"
using namespace std;
using namespace Eigen;


namespace optimizer {
    namespace snopt {
#include <string.h>
#include <assert.h>
//#include <FL/Fl.H>

        using namespace std;

		// Change made for SNOPT 7 (only load the spec once).
		static long spec_loaded = 0;

        SnoptInterface* SnoptInterface::mRef = NULL;


        SnoptInterface::SnoptInterface(long constr_tot, long coef_tot, long nonlin_constr_tot,
                                       long nonlin_obj, long nonlin_jac, long *c_eqns, long has_obj,
                                       VVD J, VVB JMap, std::vector<double> *constraints,
                                       double *objective, std::vector<double> *gradient,
                                       SnoptInterface::updateFunc update_f, void *update_d) {
            mHasObjective = has_obj;
            mNumConstr = constr_tot;
            mNumCoef = coef_tot;
            mNumNonlinConstr = nonlin_constr_tot;
            mNumNonlinObjCoef = nonlin_obj;
            mNumNonlinJacCoef = nonlin_jac;

            // init constraint type for each constraint
            if(mNumConstr != 0)
                mConstrEqns = new long[mNumConstr];
            else
                mConstrEqns = NULL;

            for(long i = 0; i < mNumConstr; i++)
                mConstrEqns[i] = c_eqns[i];

            mSolverX = new double[mNumCoef];
            mProblemX = new double[mNumCoef];
	
            mdConstrdCoef = J;
            mCoefMap = JMap;
            mConstr = constraints;
  
            mObj = objective;
            mdObjdCoef = gradient;

            mUpdateFunc = update_f;
            mUpdateData = update_d;
	
            mBoundsLo = NULL;
            mBoundsHi = NULL;

            mOutput = 0;
            mSum = 0;
            mCheckTerm = true;
            mTermination = false;
            mAbnormal = NONE;
            mBreak = -1;

            mRef = this;
        }

        SnoptInterface::SnoptInterface(long has_obj, VVD J, VVB JMap, std::vector<double> *constraints,
                                       double *objective, std::vector<double> *gradient,
                                       SnoptInterface::updateFunc update_f, void *update_d) {
            mHasObjective = has_obj;

            mConstrEqns = NULL;
            mSolverX = NULL;
            mProblemX = NULL;
	
            mdConstrdCoef = J;
            mCoefMap = JMap;
            mConstr = constraints;
  
            mObj = objective;
            mdObjdCoef = gradient;

            mUpdateFunc = update_f;
            mUpdateData = update_d;

            mOutput = 0;
            mSum = 0;
            mCheckTerm = true;
            mTermination = false;
            mAbnormal = NONE;
            mBreak = -1;


            mRef = this;
        }

        SnoptInterface::~SnoptInterface() {

            delete[] mConstrEqns;
            delete[] mSolverX;
            delete[] mProblemX;
            // delete[] lo_bounds;
            // delete[] hi_bounds;

            mRef = NULL;
        }

        void SnoptInterface::clear(long mask, long compute_derivs) {
            if(mask & SnoptInterface::Obj) {
                *mObj = 0.0;
                if (compute_derivs) {
                    for(unsigned long i = 0; i < mdObjdCoef->size(); i++) {
                        mdObjdCoef->at(i) = 0.0;
                    }
                }
            }

            if(mask & SnoptInterface::Constr) {
                for(unsigned long i = 0; i < mConstr->size(); i++)
                    mConstr->at(i) = 0.0;
                if(compute_derivs) {
                    for(unsigned long i = 0; i < mdConstrdCoef->size(); i++)
                        for(unsigned long j = 0; j < mdConstrdCoef->at(i)->size(); j++)
                            mdConstrdCoef->at(i)->at(j) = 0.0;
                    for(unsigned long i = 0; i < mCoefMap->size(); i++)
                        for(unsigned long j = 0; j < mCoefMap->at(i)->size(); j++)
                            mCoefMap->at(i)->at(j) = 0;
                }

            }
        }


        void SnoptInterface::resizeJacobian(long coef_tot, long nonlin_coef_tot,
                                            long constr_tot, long nonlin_constr_tot) {
            mNumConstr = constr_tot;
            mNumNonlinConstr = nonlin_constr_tot;

            mNumCoef = coef_tot;
            mNumNonlinObjCoef = nonlin_coef_tot;
            mNumNonlinJacCoef = nonlin_coef_tot;

            // modify constraint type for each constraint
            if (mConstrEqns != NULL)
                delete[] mConstrEqns;

            if(mNumConstr != 0)
                mConstrEqns = new long[mNumConstr];
            else
                mConstrEqns = NULL;

            for(long i = 0; i < mNumConstr; i++)
                mConstrEqns[i] = 0;

            mConstr->resize(mNumConstr);
  
            for(unsigned long i = 0; i < mdConstrdCoef->size(); i++)
                delete mdConstrdCoef->at(i);

            mdConstrdCoef->resize(mNumConstr);
  
            for(long i = 0; i < mNumConstr; i++){
                mdConstrdCoef->at(i) = new std::vector<double>;
                mdConstrdCoef->at(i)->resize(mNumCoef);
            }

            for(unsigned long i = 0; i < mCoefMap->size(); i++)
                delete mCoefMap->at(i);
  
            mCoefMap->resize(mNumConstr);
            for(long i = 0; i < mNumConstr; i++){
                mCoefMap->at(i) = new std::vector<bool>;
                mCoefMap->at(i)->resize(mNumCoef);
            }

            mdObjdCoef->resize(mNumCoef);

            if(mSolverX)
                delete[] mSolverX;
            if(mProblemX)
                delete[] mProblemX;

            mSolverX = new double[mNumCoef];
            mProblemX = new double[mNumCoef];

        }

        void SnoptInterface::update(long mask, long compute_derivs, double *x) {
            for(long i = 0; i < mNumCoef; i++){
                mProblemX[i] = x[i];
            }

            mUpdateFunc(mask, compute_derivs, mProblemX, mUpdateData);
        }

        void SnoptInterface::updateSolverX() {
            for(long i = 0; i < mNumCoef; i++)
                mSolverX[i] = mProblemX[i];
        }

        void SnoptInterface::scaleValues(long mask, long compute_derivs) {
            if(mask & SnoptInterface::Obj){
                if(compute_derivs) 
                    for(long i = 0; i < mNumCoef; i++){
                        mdObjdCoef->at(i) *= mCoefScale[i];
                    }
            }

            if (mask & SnoptInterface::Constr) {
                for(long i = 0; i < mNumConstr; i++)
                    mConstr->at(i) *= mConstrScale[i];
                if (compute_derivs) {
                    for(long i = 0; i < mNumConstr; i++)
                        for(long j = 0; j < mNumCoef; j++){
                            double scl = mConstrScale[i] * mCoefScale[j];
                            mdConstrdCoef->at(i)->at(j) *= scl;
                        }
                }
            }
        }

        extern "C" {
            void sninit_(long *iprint, long *isum, char *cw, long *lencw,  
                        long *iw, long *leniw, double *rw, long *lenrw);
            void snopt_(char *start, long *m, long *n, long *ne, 
                       long *nName, long *nnCon, long *nnObj, long *nnJac, 
                       long *iObj, double *ObjAdd, char *Prob, 
                       void (*funCon)(long *mode, long *nnCon, long *nnJac, long *neJac, 
                                      double *x, double *fCon, double *gCon, long *nState,
                                      char *cu, long *lencu, long *iu, long *leniu, 
                                      double *ru, long *lenru),
                       void (*funObj)(long *mode, long *nnObj, double *x, 
                                      double *fObj, double *gObj, long *nState,
                                      char *cu, long *lencu, long *iu, long *leniu, 
                                      double *ru, long *lenru),
                       double *a, long *ha, long *ka, double *bl, double *bu, 
                       char *Names, long *hs, double *xs, double *pi, double *rc, 
                       long *inform, long *mincw, long *miniw, long *minrw, 
                       long *nS, long *nInf, double *sInf, double *Obj, 
                       char *cu,long *lencu, long *iu,long *leniu, double *ru,long *lenru,
                       char *cw,long *lencw, long *iw,long *leniw, double *rw,long *lenrw,
                       long start_len);
            void snspec_(long *ispecs, long *inform, char *cw, long *lencw,  
                        long *iw, long *leniw, double *rw, long *lenrw);


            void
            s1user_(long *iAbort, char *MjrMsg, long *KTcond,
                   long *m, long *n, long *nb, long *nR, long *nS,
                   long *nMajor, long *nMinor, long *nSwap,
                   double *condHz, double *duInf, double *emaxS, double *fObj, 
                   double *fMrt, double gMrt, double *PenNrm, double *prInf, double *step,
                   double *vimax, double *dxnrm, double *dxrel,
                   long *ne, long *nka, double *a, long *ha, long *ka,
                   long *hs, double *bl, double *bu, double *pi, double *rc, double *xs, 
                   char *cu, long *lencu, long *iu, long *leniu, double *ru, long *lenru, 
                   char *cw, long *lencw, long *iw, long *leniw, double *rw, long *lenrw)
            {

                SnoptInterface *s = SnoptInterface::mRef;
                /*
                char *found;
                  if((found = strstr(MjrMsg, "i")) != NULL){
                  cout << "Infeasible QP found" << endl;
                  s->mAbnormal = SnoptInterface::INFEASIBLE;
                  s->mTermination = true;
                  SnoptInterface::checkTermination(iAbort, xs);
      
                  }
                */
                /*    if((found = strstr(MjrMsg, "n")) != NULL){
                      cout << "Hessian does not update" << endl;
                      s->mAbnormal = SnoptInterface::HESSIAN_UPDATE;
                      }
                      if((found = strstr(MjrMsg, "sR")) != NULL){
                      cout << "Hessian reset abnormally" << endl;
                      s->mAbnormal = SnoptInterface::HESSIAN_RESET;
                      }*/
                if(s->mCheckTerm)
                    SnoptInterface::checkTermination(iAbort, xs);
		
                if(s->mBreak == *nMajor){
                    s->mTermination = true;
                    SnoptInterface::checkTermination(iAbort, xs);
                }

            }

        } // extern "C"

		SnoptInterface::Return SnoptInterface::solve(double *x, double *lo_bounds,
                                                     double *hi_bounds, long unit) {
            //	printf("Solver: mObj=%d, nconstrs=%d, ncoefs=%d, nl_constrs=%d, nl_coefs=%d\n"
            //		, mHasObjective, mNumConstr, mNumCoef, 
            //	mNumNonlinConstr, nonlin_coef_total);

            mAbnormal = NONE;
            mTermination = false;

            long constr_count = (1 > mNumConstr)? 1 : mNumConstr;
            long nm = mNumCoef + constr_count;

            for(long i = 0; i < mNumCoef; i++)
                mProblemX[i] = x[i];

            updateSolverX();
            double *xs = new double[nm];
            for(long i = 0; i < mNumCoef; i++)
                xs[i] = mSolverX[i];

            double *bu = new double[nm];
            double *bl = new double[nm];
            long *hs = new long[nm];
            double *pi = new double[constr_count];
            // set the bounds for the unknowns, and init the unknowns vector
            for(long i = 0; i < mNumCoef; i++){
                bl[i] = lo_bounds[i];
                bu[i] = hi_bounds[i];
                hs[i] = 2;
            }

            // set bounds for the constraints
            for(long i = 0; i < mNumConstr; i++){
                if(mConstrEqns[i] == 0){
                    bl[mNumCoef + i] = -x[mNumCoef + i];
                    bu[mNumCoef + i] = x[mNumCoef + i];
                }else if(mConstrEqns[i] == 1){
                    bl[mNumCoef + i] = x[mNumCoef + i];
                    bu[mNumCoef + i] = x[mNumCoef + i] + 1e7;
                }else if(mConstrEqns[i] == 2){
                    bl[mNumCoef + i] = -1e7 + x[mNumCoef + i];
                    bu[mNumCoef + i] = -0.0 + x[mNumCoef + i];
                }else if(mConstrEqns[i] == 3){
                    bl[mNumCoef + i] = -1e-2 + x[mNumCoef + i];
                    bu[mNumCoef + i] = 1e-2 + x[mNumCoef + i];
                }else if(mConstrEqns[i] == 4){
                    bl[mNumCoef + i] = -1e-2 + x[mNumCoef + i];
                    bu[mNumCoef + i] = 1e-2 + x[mNumCoef + i];
                }else if(mConstrEqns[i] = 5){
                    bl[mNumCoef + i] = -1e3 + x[mNumCoef + i];
                    bu[mNumCoef + i] = 1e3 + x[mNumCoef + i];
                }

                pi[i] = 0;
                xs[mNumCoef + i] = x[mNumCoef + i];
                hs[mNumCoef + i] = 2;
            }

            if(mNumConstr == 0){
                pi[0] = 0;
                xs[mNumCoef] = 0.0;
                bl[mNumCoef] = -1.e7;
                bu[mNumCoef] = 1.e7;
            }
            double *rc = new double[nm];

            // initialize best solution and update function so that jacobian
            // can be frozen
            update(SnoptInterface::Obj | SnoptInterface::Constr, 1, mSolverX);

            // set the jacobian
            double *a = NULL;
            long *ha = NULL;
            long *ka = NULL;
            fillUpSnoptFormat(mdConstrdCoef, &a, &ha, &ka);

            static long lencw = 1000 * sizeof(char[8]);
            static long leniw = 800 * nm;
            static long lenrw = 6400 * nm;
            static char* cw = new char[lencw];
            static long* iw = new long[leniw];
            static double* rw = new double[lenrw];

            long iprint = mOutput;
            long ispec = unit; //if spcname is not specified, snopt will load in fort.4 as default
            long isum = mSum; // 5 == stdin

//  	fprintf(stderr,"cw: %d, iw: %d, rw: %d\n",cw,iw,rw);

            long inform; 
            if (unit != 4) {
				spec_loaded = 0;	//reset when a spacetime problem is fomulated
				lencw = 1000 * sizeof(char[8]);
				leniw = 800 * nm;
				lenrw = 6400 * nm;
				if(cw)
					delete[] cw;
				if(iw)
					delete[] iw;
				if(rw)
					delete[] rw;

				cw = new char[lencw];
				iw = new long[leniw];
				rw = new double[lenrw];	
            }

			if (!spec_loaded) {
				sninit_(&iprint, &isum, cw, &lencw, iw, &leniw, rw, &lenrw);
				snspec_(&ispec, &inform, cw, &lencw, iw, &leniw, rw, &lenrw);
				spec_loaded = 1;
            }

            //	char *startup = "Warm";
            char startup[16] = "Cold";
            long m = constr_count;
            long n = mNumCoef;
            long ne = 0;
            for(long j = 0; j < mNumCoef; j++){
                long nonZeroCount = sparseCount(j);	
                ne += (1 > nonZeroCount) ? 1 : nonZeroCount;
            }

            long nName = 1;
            long nnCon = mNumNonlinConstr;
            long nnObj = mNumNonlinObjCoef;
            long nnJac = (nnCon == 0) ? 0 : mNumNonlinJacCoef;
            long iObj = 0;
            double ObjAdd = 0;
            char problem_name[16] = "MOM";
            char names[] = " ";
            long mincw, miniw, minrw;
            long nS;
            long nInf;
            double sInf, Obj;

            snopt_(startup, &m, &n, &ne, &nName, &nnCon, &nnObj, &nnJac, 
                  &iObj, &ObjAdd, problem_name,
                  SnoptInterface::snoptJac, SnoptInterface::snoptObj,  
                  a, ha, ka, bl, bu, names, hs, xs, pi, rc,
                  &inform, &mincw, &miniw, &minrw, &nS, &nInf, &sInf, &Obj,
                  cw, &lencw, iw, &leniw, rw, &lenrw,
                  cw, &lencw, iw, &leniw, rw, &lenrw,
                  strlen(startup));

            mReturnedObj = Obj;

            update(SnoptInterface::Obj | SnoptInterface::Constr, 1, xs);

            for(long i = 0; i < mNumCoef; i++)
                x[i] = xs[i];

            delete[] xs;
            delete[] bu;
            delete[] bl; 
            delete[] hs;
            delete[] pi;
            delete[] rc;
            //  delete[] cw;
            //delete[] iw; 
            //delete[] rw;

            delete[] a;
            delete[] ha;
            delete[] ka;
  
            // cout << "objective = " << mReturnedObj << endl;

            switch(inform){
            case 0:
                return SnoptInterface::Solution;
            case 1:
                return SnoptInterface::Infeasible;
            case 3:
                return SnoptInterface::Stop;
            case 6:
            case 12:
                return SnoptInterface::UserStop;
            default:
                return SnoptInterface::Error;
            };	

        }

/* ARGSUSED */
        void  SnoptInterface::snoptObj(long *mode, long *nn_obj, double *x, 
                                    double *f_obj, double *g_obj, long *nstate, 
                                    char *cu, long *lencu, 
                                    long *iu, long *leniu, 
                                    double *ru, long *lenru) {

            SnoptInterface *s = SnoptInterface::mRef;
            assert(s != NULL);

            s->update(SnoptInterface::Obj, *mode, x);

            *f_obj = s->mObj[0];
            if(*mode == 2){
                for(long i = 0; i < *nn_obj; i++){
                    g_obj[i] = s->mdObjdCoef->at(i);
                }
            }
        }

/* ARGSUSED */
        void SnoptInterface::snoptJac(long *mode, long *nn_con, long *nn_jac, long *ne_jac,
                                   double *x, double *f_con, double *g_con, long *nstate, 
                                   char *cu, long *lencu, 
                                   long *iu, long *leniu, 
                                   double *ru, long *lenru) {
            SnoptInterface *s = SnoptInterface::mRef;
            assert(s != NULL);
            if(s->mNumConstr == 0){
                f_con[0] = 0.0;
                return;
            }

            s->update(SnoptInterface::Constr, *mode, x);

            for(long i = 0; i < *nn_con; i++){
                f_con[i] = s->mConstr->at(i);
            }


            if(*mode == 2){
                long nElt = 0;
                for(long j = 0; j < s->mNumCoef; j++){
                    if(s->sparseCount(j) == 0){
                        g_con[nElt++] = 0.0;
                        continue;
                    }
                    for(long i = 0; i < *nn_con; i++){
                        if(s->mCoefMap->at(i)->at(j))
                            g_con[nElt++] = s->mdConstrdCoef->at(i)->at(j);
                    }
                }
            }
        }

        void SnoptInterface::fillUpSnoptFormat(VVD jacobian, double **a, long **ha, long **ka) {
            //	Count up the nonzero elements and allocate memory for a, ha, and ka
            long nElts = 0;
            long cols = mNumCoef;
            long *nonZeroInCol = new long[mNumCoef];

            for(long j = 0; j < cols; j++){
                nonZeroInCol[j] = sparseCount(j);
                nElts += (1 > nonZeroInCol[j]) ? 1 : nonZeroInCol[j];
            }

            if(*a != NULL)
                delete[] *a;
            if(*ha != NULL)
                delete[] *ha;
            if(*ka != NULL)
                delete[] *ka;
            *a = new double[nElts];
            *ha = new long[nElts];
            *ka = new long[mNumCoef + 1];

            //	Fillup the SNOPT sparse structure
            nElts = 0;

            for(long j = 0; j < cols; j++){
                (*ka)[j] = nElts + 1;	 //Fortran starts from 1
                if(nonZeroInCol[j] == 0){ //Deal with empty column
                    (*a)[nElts] = 0.0;
                    (*ha)[nElts] = 1;
                    nElts++;
                    continue;
                }

                for(long i = 0; i < mNumConstr; i++){
                    if(mCoefMap->at(i)->at(j)){
                        (*a)[nElts] = jacobian->at(i)->at(j);
                        (*ha)[nElts] = i + 1;	//Fortran starts from 1
                        nElts++;
                    }
                }
            }

            (*ka)[mNumCoef] = nElts + 1;	//Last entry is special	
            delete[] nonZeroInCol;
        }

//long SnoptInterface::sparseCount()
//{
//	long numNonZero = 0;
//	for(long i = 0; i < mNumConstr; i++)
//		for(long j = 0; j < mNumCoef; j++)
//			if(mCoefMap->at(i)->at(j))
//				numNonZero++;
//	return numNonZero;
//}

        long SnoptInterface::sparseCount(long col) {
            long numNonZero = 0;
            for(long i = 0; i < mNumConstr; i++)
                if(mCoefMap->at(i)->at(col))
                    numNonZero++;
	
            return numNonZero;
        }


        void SnoptInterface::checkTermination(long *iAbort, double *xs) {
            SnoptInterface *s = SnoptInterface::mRef;
            // cout << "check " << s->mTermination << endl;
            if(s->mTermination){
                *iAbort = 1;
                s->mTermination = false;
            }
        }
        
    } // namespace snopt
} // namespace optimizer
