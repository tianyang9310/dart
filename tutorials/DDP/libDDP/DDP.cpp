#include "DDP.h"

namespace DDP_NSpace
{
VectorXd gxTemplate(VectorXd xtT, int iT, MatrixXd uT, MatrixXd xT, vector<MatrixXd> kT, vector<MatrixXd> KT, double alphaT);

DDP::DDP(int T, function<VectorXd(const VectorXd, const VectorXd)> StepDynamics, function<Scalar(const VectorXd, const VectorXd)> StepCost, function<Scalar(const VectorXd)> FinalCost, vector<tuple<MatrixXd, MatrixXd, MatrixXd>> LQR, tuple<VectorXd, VectorXd, int> StateBundle):
    StepDynamics(StepDynamics),StepCost(StepCost),FinalCost(FinalCost),
    T(T),Vx(T),Vxx(T),k(T),K(T),Quu_inv(T),gx(T)
{
// --------------------------------------------------
// constant initialization
    isLQR       = false;
    if (!LQR.empty())
    {
        isLQR   = true;
        Q       = get<0>(LQR[0]);
        R       = get<1>(LQR[0]);
        Qf      = get<2>(LQR[0]);
    }
    if (isLQR)
    {
        cout<<"Solving LQR problem..."<<endl;
    }
    else
    {
        cout<<"Solving non-LQR problem"<<endl;
    }

    x0 = get<0>(StateBundle);
    xd = get<1>(StateBundle);
    mu_default = 0;    
    mu    = mu_default;
    alpha = 1;
// matrix initialization
    x_dim = x0.rows();
    u_dim = get<2>(StateBundle); 
    fx.resize(x_dim,x_dim);
    fu.resize(x_dim,u_dim);
    Cx.resize(x_dim,1);
    Cx.resize(u_dim,1);
    Cxx.resize(x_dim,x_dim);
    Cuu.resize(u_dim,u_dim);
    Cux.resize(u_dim,x_dim);
// memory allocation
    x       = MatrixXd::Zero(x_dim,T);
    u       = MatrixXd::Zero(u_dim,T);
    C       = VectorXd::Zero(T);
    x_new   = MatrixXd::Zero(x_dim,T);
    u_new   = MatrixXd::Zero(u_dim,T);
    C_new   = VectorXd::Zero(T);
// --------------------------------------------------
// produce initial trajectory
    //u         = MatrixXd::Random(u_dim,T)*150;
    u.col(T-1) = VectorXd::Constant(u_dim,nan("0"));

    /*x       =*/ TrajGenerator(/*x0, u*/);
// --------------------------------------------------
    dV.setZero();
    for (int i=0;i<T;i++)
    {
        Vx[i].resize(x_dim,1);
        Vxx[i].resize(x_dim,x_dim);
        k[i].resize(u_dim,1);
        K[i].resize(u_dim,x_dim);
        Quu_inv[i].resize(u_dim,u_dim);

        Vx[i].setZero();
        Vxx[i].setZero();
        k[i].setZero();
        K[i].setZero();
        Quu_inv[i].setZero();
    }
}

void DDP::TrajGenerator()
{
// --------------------------------------------------
//  Whole Trajectory Generator
//  Computing _x according to _x0 and _u using step dynamics
// --------------------------------------------------

    x.col(0) = x0;

    for (int i=0; i<T-1; i++)
    {
        x.col(i+1) = StepDynamics(x.col(i), u.col(i));
        C.row(i) = StepCost(x.col(i),u.col(i));
    }
    C.row(T-1)  = FinalCost(x.col(T-1));
}

void DDP::trajopt()
{
// one iteration of DDP
// --------------------------------------------------
// backward pass
    bool diverge = true;
    while (diverge)
    {
        diverge = backwardpass();
        if (diverge)
        {
            mu +=5;
        }
    }
    mu = mu_default;

// forward  pass
    bool forward_done = false;
    while(!forward_done)
    {
        forwardpass();

        cout<<"One forward iteration finishes..."<<endl;

        double dCost = C.sum() - C_new.sum();
        double expected = -alpha*(dV[0]+alpha*dV[1]);
        double z;
        if (expected>0)
        {
            z = dCost/expected;
            cout<<"dCost: "<<dCost<<" expected: "<<expected<<endl;
            dtmsg<<"positive expected reduction "<<z<<endl;
        }
        else
        {
            z = 2*(dCost >= 0)-1;
            dtmsg<<"non-positive expected reduction "<<z<<endl;
        }
        if (z>=0)
        {
            forward_done = true;
            break;
        }
        else
        {
            alpha *= pow(10,-0.3);
        }
    }
    alpha = 1;
// --------------------------------------------------
    x = x_new;
    u = u_new;
    C = C_new;
    dtmsg<<"updated x, u and c..."<<endl;
// --------------------------------------------------
    cout<<"Current cost is "<<C.sum()<<endl;
}

bool DDP::backwardpass()
{
//  variable initialization
    dtmsg<<"Backward pass starting..."<<endl;
    bool localDiverge = false;
    dV.setZero();

    if (isLQR)
    {
        Vx[T-1]     = (Qf*(x.col(T-1)-xd));
        Vxx[T-1]    = Qf;
    }
    else
    {
        Vx[T-1]  = (FiniteDiff(FinalCost,x.col(T-1))).transpose();
        Vxx[T-1] = FiniteDiff([=](VectorXd Var)->MatrixXd{
                        return FiniteDiff(FinalCost,Var);},
                        x.col(T-1));
    }

    for (int i=T-2;i>=0;i--)
    {
        Derivative(x.col(i),u.col(i));

        MatrixXd Qx(x_dim,1);    
        MatrixXd Qu(u_dim,1);    
        MatrixXd Qxx(x_dim,x_dim);   
        MatrixXd Quu(u_dim,u_dim);   
        MatrixXd Qux(u_dim,x_dim);   
        MatrixXd Quu_reg(u_dim,u_dim);       
        MatrixXd Qux_reg(u_dim,x_dim);       

        Qx      = Cx + fx.transpose()*Vx[i+1];
        Qu      = Cu + fu.transpose()*Vx[i+1];
        Qxx     = Cxx + fx.transpose()*Vxx[i+1]*fx;
        Quu     = Cuu + fu.transpose()*Vxx[i+1]*fu;
        Qux     = Cux + fu.transpose()*Vxx[i+1]*fx;
        MatrixXd uEye(u_dim,u_dim);
        uEye.setIdentity();
        Quu_reg = Quu + mu*uEye;
        Qux_reg = Qux;
// --------------------------------------------------
//      TODO: change the analyze to cholesky decomposition
//            to find out whether Quu is PD
        if (Quu_reg(0)<=0)
        {
            localDiverge = true;
            dtmsg<<"Diverge occurs in the backward pass"<<endl;
            return localDiverge;
        }
        //k[i]  = -Qu/Quu_reg(0);
        //K[i]  = -Qux_reg/Quu_reg(0);
        k[i]    = Quu_reg.ldlt().solve(-Qu);
        K[i]    = Quu_reg.ldlt().solve(-Qux_reg);
        Quu_inv[i]
                = Quu_reg.inverse();

        dV     += (Vector2d() << k[i].transpose()*Qu, 0.5*k[i].transpose()*Quu*k[i]).finished();
        Vx[i]   = Qx+K[i].transpose()*Quu*k[i]+K[i].transpose()*Qu+Qux.transpose()*k[i];
        Vxx[i]  = Qxx + K[i].transpose()*Quu*K[i]+K[i].transpose()*Qux+Qux.transpose()*K[i];
        Vxx[i]  = 0.5*(Vxx[i]+Vxx[i].transpose());
    }
    return localDiverge;
}


void DDP::forwardpass()
{
    dtmsg<<"Forward pass starting..."<<endl;
    x_new.col(0) = x0;

    for (int i=0; i<T-1; i++)
    {
        u_new.col(i) = u.col(i) + alpha*k[i] + K[i]*(x_new.col(i)-x.col(i));
        x_new.col(i+1) = StepDynamics(x_new.col(i), u_new.col(i));
        C_new.row(i) = StepCost(x_new.col(i),u_new.col(i));

        gx[i] = bind(gxTemplate, placeholders::_1,i,u,x,k,K,alpha);
    }
    u_new.col(T-1) = VectorXd::Constant(u_dim,nan("0"));
    C_new.row(T-1) = FinalCost(x_new.col(T-1));
}

void DDP::Derivative(const VectorXd & _xi, const VectorXd & _ui)
{
// compute fx, fu, Cx, Cu, Cxx, Cuu, Cux according to x_i and u_i
// Cx, Cu, Cxx, Cuu, Cux are computed according to analytic solution
// whether has delta_t here, I think it shouldn't. Because all we consider
// is discreteized system.
    if (isLQR)
    {
        Cx = (Q*(_xi - xd));
        Cu = (R*_ui);
        Cxx = Q;
        Cuu = R;
        Cux.setZero();
        
// ----------------------------------------
//      cout<<"#########  LQR   #################"<<endl;
//      cout<<Cx<<endl;
//      cout<<Cu<<endl;
//      cout<<Cxx<<endl;
//      cout<<Cuu<<endl;
//      cout<<Cux<<endl;
//      cout<<"#########finite diff   #################"<<endl;
//      MatrixXd Cxu_bundle;
//      Cxu_bundle  = FiniteDiff([=](VectorXd Var){
//                return StepCost(Var.head(_xi.rows()), Var.tail(_ui.rows()));},
//               (VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
//      Cx = (Cxu_bundle.leftCols(_xi.rows())).transpose();
//      Cu = (Cxu_bundle.rightCols(_ui.rows())).transpose();
//
//      MatrixXd Cxxuu_bundle;
//
//      auto Cxu_FD = [=](VectorXd __xi, VectorXd __ui){
//      return    
//          FiniteDiff([=](VectorXd Var){
//              return StepCost(Var.head(__xi.rows()), Var.tail(__ui.rows()));},
//              (VectorXd(__xi.rows()+__ui.rows()) << __xi, __ui ).finished())
//      ;};
//
//      Cxxuu_bundle = FiniteDiff([=](VectorXd Var){
//              return Cxu_FD(Var.head(_xi.rows()),Var.tail(_ui.rows()));},
//              (VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
//
//      Cxx = Cxxuu_bundle.topLeftCorner(_xi.rows(),_xi.rows());
//      Cuu = Cxxuu_bundle.bottomRightCorner(_ui.rows(),_ui.rows());
//      Cux = Cxxuu_bundle.bottomLeftCorner(_ui.rows(),_xi.rows());
//
//      cout<<Cx<<endl;
//      cout<<Cu<<endl;
//      cout<<Cxxuu_bundle<<endl;
//      cin.get();
// ----------------------------------------
    }
    else
    {
        MatrixXd Cxu_bundle;
        Cxu_bundle  = FiniteDiff([=](VectorXd Var)->Scalar{
                  return StepCost(Var.head(_xi.rows()), Var.tail(_ui.rows()));},
                 (VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
        Cx = (Cxu_bundle.leftCols(_xi.rows())).transpose();
        Cu = (Cxu_bundle.rightCols(_ui.rows())).transpose();

        MatrixXd Cxxuu_bundle;

        auto Cxu_FD = [=](VectorXd __xi, VectorXd __ui)->MatrixXd{
        return    
            FiniteDiff([=](VectorXd Var)->Scalar{
                return StepCost(Var.head(__xi.rows()), Var.tail(__ui.rows()));},
                (VectorXd(__xi.rows()+__ui.rows()) << __xi, __ui ).finished())
        ;};

        Cxxuu_bundle = FiniteDiff([=](VectorXd Var)->MatrixXd{
                return Cxu_FD(Var.head(_xi.rows()),Var.tail(_ui.rows()));},
                (VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());

        Cxx = Cxxuu_bundle.topLeftCorner(_xi.rows(),_xi.rows());
        Cuu = Cxxuu_bundle.bottomRightCorner(_ui.rows(),_ui.rows());
        Cux = Cxxuu_bundle.bottomLeftCorner(_ui.rows(),_xi.rows());

    }
    
// fx, fu are computed according to finite difference
    MatrixXd fxu_bundle;
    fxu_bundle = FiniteDiff([this](VectorXd Var)->VectorXd{
            return (StepDynamics(Var.head(x_dim), Var.tail(u_dim))).eval();}, 
            (VectorXd(x_dim+u_dim) << _xi, _ui).finished());
    fx  = fxu_bundle.leftCols(x_dim);
    fu  = fxu_bundle.rightCols(u_dim);
}

VectorXd gxTemplate(VectorXd xtT, int iT, MatrixXd uT, MatrixXd xT, vector<MatrixXd> kT, vector<MatrixXd> KT, double alphaT)
{
    // cout<<"u.col(i) + alpha*k[i] + K[i]*(xt - x.col(i))"<<endl;
    // cout<<"u.col("<<iT<<") = "<<uT.col(iT)<<endl;
    // cout<<"alpha = "<<alphaT<<endl;
    // cout<<"k["<<iT<<"] = "<<kT[iT]<<endl;
    // cout<<"K["<<iT<<"] = "<<KT[iT]<<endl;
    // cout<<"xt = "<<xtT.transpose()<<endl;
    // cout<<"x.col("<<iT<<") = "<<xT.col(iT).transpose()<<endl;
    return (KT[iT]*(xtT - xT.col(iT)) + uT.col(iT) + alphaT*kT[iT]).eval();
}

void DDP::setMu()
{
    mu_default = 0.05;
    mu = mu_default;
}


}
