#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <ode/ode.h>

class SimulationState {
  public:
    double fT;
    double fX1, fY1, fZ1;
    double fX2, fY2, fZ2;
};

class Simulation {
  public:
    Simulation();
    ~Simulation();
    
    void AdvanceTo(double t);
    SimulationState GetCurrentState();
    
    dWorldID fWorld;
    dBodyID fBall1, fBall2;
    
    double fTime;
    
    static const double TIMESTEP;
};

#endif
