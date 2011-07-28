#ifndef MSIM_SIMULATION_H
#define MSIM_SIMULATION_H

#include "Vector.h"

class SimulationState {
  public:
    virtual void Draw() const = 0;
    virtual Vector3 GetCenter() const { return Vector3::Null; }
    
    // Interface for data display
    virtual double GetData(int) const
        { return std::numeric_limits<double>::quiet_NaN(); }
};

class Simulation {
  public:
    Simulation() {};
    virtual ~Simulation() {};
    
    virtual double GetTimestep() = 0;    // in units of seconds
    virtual int GetDefaultEndTime() = 0; // in units of TIMESTEPs
    
    virtual const SimulationState* GetState(int t) = 0;  // t in units of TIMESTEPs
    virtual const char* GetTitle() = 0;
    
    // Interface for data display
    virtual int GetNDataCh() const { return 0; }
    virtual const char* GetChName(int) const { return ""; }
    
    // Interface for asynchronous simulation
    virtual void StartSimulation() = 0;
    virtual void ReadData() = 0;
    virtual bool finished() = 0;
    virtual int GetDescriptor() = 0;
};

#endif
