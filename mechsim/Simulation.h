#ifndef __SIMULATION_H__
#define __SIMULATION_H__

class SimulationState {
  public:
    virtual void Draw() const = 0;
};

class Simulation {
  public:
    Simulation() {};
    virtual ~Simulation() {};
    
    virtual double GetTimestep() = 0;    // in units of seconds
    virtual int GetDefaultEndTime() = 0; // in units of TIMESTEPs
    
    virtual const SimulationState* GetState(int t) = 0;  // t in units of TIMESTEPs
    virtual const char* GetTitle() = 0;
};

#endif
