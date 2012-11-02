#ifndef MSIM_SIMULATION_H
#define MSIM_SIMULATION_H

#include <Eigen/Dense>

class SimulationState {
  public:
    virtual void Draw(int mode) const = 0;
    virtual Eigen::Vector3d GetCenter() const { return Eigen::Vector3d::Zero(); }
    
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
    
    virtual const char* GetTitle() = 0;
    
    // Interface for draw modes
    virtual int GetNDrawModes() const { return 1; }
    virtual const char* GetDrawModeName(int) { return "<Default>"; }
    
    // Interface for data display
    virtual int GetNDataCh() const { return 0; }
    virtual const char* GetChName(int) const { return ""; }
};

#endif
