#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

// class containing objects to be simulated
class Simulator {
public:
    Simulator() {}          //dflt ctor - nothing goes in here!
    Simulator(int _sysType);
        
    void simulate();

    void integrate();


    int getNumParticles() {return mParticles.size();}

    Particle* getParticle(int index) {return &mParticles[index];}

    double getTimeStep() { return mTimeStep;}
    
    int getFrameNum() { return mFrameNumber; }
    
    void reset();
    void ClearForce();
    void ApplyForce();
    void ApplyForce_P1();
    void ApplyForce_P2();
    void ApplyExtForce();
    void ApplyGravityForce();
    void ApplyConstraintForce();
    void DerivativeEval();

    void analyticSolve(Particle& p);
    void fwdEulerSolve(Particle& p);
    void midPointSolve(Particle& p);
    void ImplicitEurlerSolve(Particle& p);
    void RK4Solve(Particle& p); 

    void InitVelocity();

    void part1Galileo();

    void part2TinkerToy();


    Eigen::Vector3d view_translation = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector2d view_rotation = Eigen::Vector2d::Zero();
    bool mouse_down = false;
    Eigen::Vector2d mouse_click_position;
    int Integrator;

// private:
    double mTimeStep;       // time step
    double mElapsedTime;    // time pased since beginning of simulation
    std::vector<Particle> mParticles;
    int sysType;
    int mFrameNumber;
    Eigen::Vector3d mInitVelocity;
    Eigen::Vector3d ExtForce;
    int flag;
    
};

#endif  // SIMULATOR_H
