#include "DDP.h"
//#define mDebug_DDP

namespace DDP_NSpace
{

DDP::DDP(int T, std::function<Eigen::VectorXd(const Eigen::VectorXd, const Eigen::VectorXd)> StepDynamics, std::function<Scalar(const Eigen::VectorXd, const Eigen::VectorXd)> StepCost, std::function<Scalar(const Eigen::VectorXd)> FinalCost, std::vector<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>> LQR, std::tuple<Eigen::VectorXd, Eigen::VectorXd, int> StateBundle):
    StepDynamics(StepDynamics),
    StepCost(StepCost),
    FinalCost(FinalCost),
    T(T),
    Vx(T),
    Vxx(T),
    k(T),
    K(T),
    Quu_inv(T),
    gx(T)
{
// --------------------------------------------------
// constant initialization
    isLQR       = false;
    if (!LQR.empty())
    {
        isLQR   = true;
        Q       = std::get<0>(LQR[0]);
        R       = std::get<1>(LQR[0]);
        Qf      = std::get<2>(LQR[0]);
    }
    if (isLQR)
    {
        std::cout<<"Solving LQR problem..."<<std::endl;
    }
    else
    {
        std::cout<<"Solving non-LQR problem"<<std::endl;
    }

    x0 = std::get<0>(StateBundle);
    xd = std::get<1>(StateBundle);
    mu_default = 0;    
    mu    = mu_default;
    alpha = 1;
// matrix initialization
    x_dim = x0.rows();
    u_dim = std::get<2>(StateBundle); 
    fx.resize(x_dim,x_dim);
    fu.resize(x_dim,u_dim);
    Cx.resize(x_dim,1);
    Cx.resize(u_dim,1);
    Cxx.resize(x_dim,x_dim);
    Cuu.resize(u_dim,u_dim);
    Cux.resize(u_dim,x_dim);
// memory allocation
    x       = Eigen::MatrixXd::Zero(x_dim,T);
    u       = Eigen::MatrixXd::Zero(u_dim,T);
    C       = Eigen::VectorXd::Zero(T);
    x_new   = Eigen::MatrixXd::Zero(x_dim,T);
    u_new   = Eigen::MatrixXd::Zero(u_dim,T);
    C_new   = Eigen::VectorXd::Zero(T);
// --------------------------------------------------
// produce initial trajectory
    //u         = Eigen::MatrixXd::Random(u_dim,T)*150;
    u.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));

    x       = TrajGenerator(x0, u);
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

#ifdef mDebug_DDP
// DDP initial data and some variable output
//  std::cout<<"Initial control sequence is"<<std::endl<<u<<std::endl;
//  std::cout<<"Initial state   sequence is"<<std::endl<<x.transpose()<<std::endl;
    std::cout<<"Initial cost is "<<C.sum()<<std::endl;
    std::cout<<"Press any key to print initial x and u to file..."<<std::endl;
//  std::cin.get();
    write2file_eigen(x,"x");
    write2file_eigen(u,"u");
    std::cout<<"Please use python script to plot figures"<<std::endl;
    std::cout<<"Press any key to continue..."<<std::endl;
    //std::cin.get();
#endif
}

Eigen::MatrixXd DDP::TrajGenerator(const Eigen::VectorXd _x0, const Eigen::MatrixXd _u)
{
// --------------------------------------------------
//  Whole Trajectory Generator
//  Computing _x according to _x0 and _u using step dynamics
// --------------------------------------------------

    Eigen::MatrixXd _x;
    _x.setZero(_x0.rows(),T);
    _x.col(0) = _x0;

    for (int i=0; i<T-1; i++)
    {
        _x.col(i+1) = StepDynamics(_x.col(i), _u.col(i));
        C.row(i) = StepCost(x.col(i),u.col(i));
    }
    C.row(T-1)  = FinalCost(x.col(T-1));
    return _x;
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
// --------------------------------------------------
#ifdef mDebug_DDP
//  backward debugging
        std::cout<<diverge<<std::endl;
        std::cout<<"Press any key to print k, K, Vx, Vxx to file..."<<std::endl;
        write2file_std(k,"k");
        write2file_std(K,"K");
        write2file_std(Vx,"Vx");
        write2file_std(Vxx,"Vxx");
        std::cin.get();
#endif
// --------------------------------------------------

// forward  pass
    bool forward_done = false;
    while(!forward_done)
    {
        forwardpass();

        std::cout<<"One forward iteration finishes..."<<std::endl;
        std::cout<<"Press any key to continue..."<<std::endl;

        double dCost = C.sum() - C_new.sum();
        double expected = -alpha*(dV[0]+alpha*dV[1]);
        double z;
        if (expected>0)
        {
            z = dCost/expected;
            std::cout<<"dCost: "<<dCost<<" expected: "<<expected<<std::endl;
            dtmsg<<"positive expected reduction "<<z<<std::endl;
            std::cout<<"Press any key to continue..."<<std::endl;
        }
        else
        {
            z = 2*(dCost > 0)-1;
            dtmsg<<"non-positive expected reduction "<<z<<std::endl;
            std::cout<<"Press any key to continue..."<<std::endl;
        }
        if (z>0)
        {
            forward_done = true;
            break;
        }
        else
        {
            alpha *= std::pow(10,-0.3);
        }
    }
    alpha = 1;
// --------------------------------------------------
    x = x_new;
    u = u_new;
    C = C_new;
// --------------------------------------------------
    std::cout<<"Current cost is "<<C.sum()<<std::endl;
#ifdef mDebug_DDP
    std::cout<<"Press any key to print x and u to file..."<<std::endl;
    //std::cin.get();
    write2file_eigen(x,"x");
    write2file_eigen(u,"u");
    std::cout<<"Please use python script to plot figures"<<std::endl;
    std::cout<<"Press any key to continue..."<<std::endl;
    //std::cin.get();
#endif
}

bool DDP::backwardpass()
{
//  variable initialization
    dtmsg<<"Backward pass starting..."<<std::endl;
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
        Vxx[T-1] = FiniteDiff([=](Eigen::VectorXd Var){
                        return FiniteDiff(FinalCost,Var);},
                        x.col(T-1));
    }

    for (int i=T-2;i>=0;i--)
    {
        Derivative(x.col(i),u.col(i));

        Eigen::MatrixXd Qx(x_dim,1);    
        Eigen::MatrixXd Qu(u_dim,1);    
        Eigen::MatrixXd Qxx(x_dim,x_dim);   
        Eigen::MatrixXd Quu(u_dim,u_dim);   
        Eigen::MatrixXd Qux(u_dim,x_dim);   
        Eigen::MatrixXd Quu_reg(u_dim,u_dim);       
        Eigen::MatrixXd Qux_reg(u_dim,x_dim);       

        Qx      = Cx + fx.transpose()*Vx[i+1];
        Qu      = Cu + fu.transpose()*Vx[i+1];
        Qxx     = Cxx + fx.transpose()*Vxx[i+1]*fx;
        Quu     = Cuu + fu.transpose()*Vxx[i+1]*fu;
        Qux     = Cux + fu.transpose()*Vxx[i+1]*fx;
        Eigen::MatrixXd uEye(u_dim,u_dim);
        uEye.setIdentity();
        Quu_reg = Quu + mu*uEye;
        Qux_reg = Qux;
// --------------------------------------------------
//      backward debugging
#ifdef mDebug_DDP
        if (i%100 == 0)
        {
            dtmsg<<" "<<i<<" step in backward pass"<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"fx "<<std::endl<<fx<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"fu "<<std::endl<<fu<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Cx "<<std::endl<<Cx<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Cu "<<std::endl<<Cu<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Cxx "<<std::endl<<Cxx<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Cuu "<<std::endl<<Cuu<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Cux "<<std::endl<<Cux<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Qx "<<std::endl<<Qx<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Qu "<<std::endl<<Qu<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Qxx "<<std::endl<<Qxx<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Quu "<<std::endl<<Quu<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Quu_reg "<<std::endl<<Quu_reg<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Qux_reg "<<std::endl<<Qux_reg<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"value function derivative "<<std::endl;
            std::cout<<"Vx "<<std::endl<<Vx[i+1]<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Vxx "<<std::endl<<Vxx[i+1]<<std::endl;
            std::cout<<"***************************************"<<std::endl;
            std::cout<<"Break point in backward pass. Press any key to continue"<<std::endl;
            //std::cin.get();
        }
#endif
// --------------------------------------------------
//      TODO: change the analyze to cholesky decomposition
//            to find out whether Quu is PD
        if (Quu_reg(0)<=0)
        {
            localDiverge = true;
            dtmsg<<"Diverge occurs in the backward pass"<<std::endl;
            return localDiverge;
        }
        //k[i]  = -Qu/Quu_reg(0);
        //K[i]  = -Qux_reg/Quu_reg(0);
        k[i]    = Quu_reg.ldlt().solve(-Qu);
        K[i]    = Quu_reg.ldlt().solve(-Qux_reg);
        Quu_inv[i]
                = Quu_reg.inverse();

        dV     += (Eigen::Vector2d() << k[i].transpose()*Qu, 0.5*k[i].transpose()*Quu*k[i]).finished();
        Vx[i]   = Qx+K[i].transpose()*Quu*k[i]+K[i].transpose()*Qu+Qux.transpose()*k[i];
        Vxx[i]  = Qxx + K[i].transpose()*Quu*K[i]+K[i].transpose()*Qux+Qux.transpose()*K[i];
        Vxx[i]  = 0.5*(Vxx[i]+Vxx[i].transpose());
    }
    return localDiverge;
}

void DDP::forwardpass()
{
    dtmsg<<"Forward pass starting..."<<std::endl;
    x_new.col(0) = x0;
    for (int i=0; i<T-1; i++)
    {
        u_new.col(i) = u.col(i) + alpha*k[i] + K[i]*(x_new.col(i)-x.col(i));
        x_new.col(i+1) = StepDynamics(x_new.col(i), u_new.col(i));
        C_new.row(i) = StepCost(x_new.col(i),u_new.col(i));


        gx[i] = [=](Eigen::VectorXd xt)
                    {
                        VectorXd gx_result=(K[i]*(xt - x.col(i)) + u.col(i) + alpha*k[i]);
                        return gx_result;
                    };

        // // debug gx
        // std::cout<<"~~~~~~~~~~~~~~~~~~~~~~"<<std::endl;
        // std::cout<<"u_new is "<<u_new.col(i)<<std::endl;
        // std::cout<<"gx is "<<gx[i](x_new.col(i))<<std::endl;
        // std::cout<<"press any key to continue...";
        // std::cin.get();
    }
    u_new.col(T-1) = Eigen::VectorXd::Constant(u_dim,std::nan("0"));
    C_new.row(T-1) = FinalCost(x_new.col(T-1));
}

void DDP::Derivative(Eigen::VectorXd _xi, Eigen::VectorXd _ui)
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
//      std::cout<<"#########  LQR   #################"<<std::endl;
//      std::cout<<Cx<<std::endl;
//      std::cout<<Cu<<std::endl;
//      std::cout<<Cxx<<std::endl;
//      std::cout<<Cuu<<std::endl;
//      std::cout<<Cux<<std::endl;
//      std::cout<<"#########finite diff   #################"<<std::endl;
//      Eigen::MatrixXd Cxu_bundle;
//      Cxu_bundle  = FiniteDiff([=](Eigen::VectorXd Var){
//                return StepCost(Var.head(_xi.rows()), Var.tail(_ui.rows()));},
//               (Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
//      Cx = (Cxu_bundle.leftCols(_xi.rows())).transpose();
//      Cu = (Cxu_bundle.rightCols(_ui.rows())).transpose();
//
//      Eigen::MatrixXd Cxxuu_bundle;
//
//      auto Cxu_FD = [=](Eigen::VectorXd __xi, Eigen::VectorXd __ui){
//      return    
//          FiniteDiff([=](Eigen::VectorXd Var){
//              return StepCost(Var.head(__xi.rows()), Var.tail(__ui.rows()));},
//              (Eigen::VectorXd(__xi.rows()+__ui.rows()) << __xi, __ui ).finished())
//      ;};
//
//      Cxxuu_bundle = FiniteDiff([=](Eigen::VectorXd Var){
//              return Cxu_FD(Var.head(_xi.rows()),Var.tail(_ui.rows()));},
//              (Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
//
//      Cxx = Cxxuu_bundle.topLeftCorner(_xi.rows(),_xi.rows());
//      Cuu = Cxxuu_bundle.bottomRightCorner(_ui.rows(),_ui.rows());
//      Cux = Cxxuu_bundle.bottomLeftCorner(_ui.rows(),_xi.rows());
//
//      std::cout<<Cx<<std::endl;
//      std::cout<<Cu<<std::endl;
//      std::cout<<Cxxuu_bundle<<std::endl;
//      std::cin.get();
// ----------------------------------------
    }
    else
    {
        Eigen::MatrixXd Cxu_bundle;
        Cxu_bundle  = FiniteDiff([=](Eigen::VectorXd Var){
                  return StepCost(Var.head(_xi.rows()), Var.tail(_ui.rows()));},
                 (Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());
        Cx = (Cxu_bundle.leftCols(_xi.rows())).transpose();
        Cu = (Cxu_bundle.rightCols(_ui.rows())).transpose();

        Eigen::MatrixXd Cxxuu_bundle;

        auto Cxu_FD = [=](Eigen::VectorXd __xi, Eigen::VectorXd __ui){
        return    
            FiniteDiff([=](Eigen::VectorXd Var){
                return StepCost(Var.head(__xi.rows()), Var.tail(__ui.rows()));},
                (Eigen::VectorXd(__xi.rows()+__ui.rows()) << __xi, __ui ).finished())
        ;};

        Cxxuu_bundle = FiniteDiff([=](Eigen::VectorXd Var){
                return Cxu_FD(Var.head(_xi.rows()),Var.tail(_ui.rows()));},
                (Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui ).finished());

        Cxx = Cxxuu_bundle.topLeftCorner(_xi.rows(),_xi.rows());
        Cuu = Cxxuu_bundle.bottomRightCorner(_ui.rows(),_ui.rows());
        Cux = Cxxuu_bundle.bottomLeftCorner(_ui.rows(),_xi.rows());

    }
    
// fx, fu are computed according to finite difference
    Eigen::MatrixXd fxu_bundle;
    fxu_bundle = FiniteDiff([=](Eigen::VectorXd Var){
            return StepDynamics(Var.head(_xi.rows()), Var.tail(_ui.rows()));}, 
            (Eigen::VectorXd(_xi.rows()+_ui.rows()) << _xi, _ui).finished());
    fx  = fxu_bundle.leftCols(_xi.rows());
    fu  = fxu_bundle.rightCols(_ui.rows());
}

void DDP::setMu()
{
    mu_default = 10;
    mu = mu_default;
}

}
