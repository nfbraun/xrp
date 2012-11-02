#ifndef MSIM_SIMRUNNER_H
#define MSIM_SIMRUNNER_H

#include "Simulation.h"

class SimRunner {
  public:
    SimRunner() {};
    virtual ~SimRunner() {};
    
    virtual double GetTimestep() = 0;    // in units of seconds
    virtual int GetDefaultEndTime() = 0; // in units of TIMESTEPs
    
    virtual const SimulationState* GetState(int t) = 0;  // t in units of TIMESTEPs
    virtual const char* GetTitle() = 0;
    
    // Interface for draw modes
    virtual int GetNDrawModes() const = 0;
    virtual const char* GetDrawModeName(int) = 0;
    
    // Interface for data display
    virtual int GetNDataCh() const = 0;
    virtual const char* GetChName(int) const = 0;
    
    // Interface for asynchronous simulation
    virtual void StartSimulation() = 0;
    virtual void ReadData() = 0;
    virtual bool finished() = 0;
    virtual int GetDescriptor() = 0;
};

#endif
