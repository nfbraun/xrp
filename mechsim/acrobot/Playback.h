#ifndef MSIM_PLAYBACK_H
#define MSIM_PLAYBACK_H

#include <Eigen/Dense>
#include "Simulation.h"
#include "CSVFile.h"

class AcroState: public SimulationState {
  public:
    double fT;
    double fPhi1, fPhi2;
    double fOmega1, fOmega2;
    double fU;
    void Draw(int) const;
    
    virtual double GetData(int ch) const
    {
        switch(ch) {
            case 0: return fPhi1;
            case 1: return fPhi2;
            case 2: return fOmega1;
            case 3: return fOmega2;
            case 4: return fU;
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    }
};

class Playback: public Simulation {
  public:
    Playback();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 20 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    int GetNDataCh() const { return 5; }
    virtual const char* GetChName(int ch) const
    {
        switch(ch) {
            case 0:  return "Phi1";
            case 1:  return "Phi2";
            case 2:  return "Omega1";
            case 3:  return "Omega2";
            case 4:  return "Control";
            default: return "<undefined>";
        }
    }
    
    void Advance();
    AcroState GetCurrentState();
    
  private:
    double GetInterpValue(int value_id, double t, int tidx);
    
    int fCurStep;
    int fCurTIdx;
    CSVFile fDataFile;
    
    static const int STEP_PER_SEC;
    static const char TITLE[];
};

#endif
