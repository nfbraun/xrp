#ifndef MSIM_SYNCSIMRUNNER_H
#define MSIM_SYNCSIMRUNNER_H

#include "SimRunner.h"
#include <vector>

// Runner for simulations that can be completed near-instantaneously
// at program startup
template<class Sim_T, class State_T>
class SyncSimRunner: public SimRunner
{
  public:
    SyncSimRunner(Sim_T* sim) : fSim(sim) { }
    
    virtual const State_T* GetState(int t)  // t in units of TIMESTEPs
    {
        if(t < 0) return 0;
        
        while((int)fCache.size() <= t) {
            fCache.push_back(fSim->GetCurrentState());
            fSim->Advance();
        }
        
        return &fCache[t];
    }
    
    // Calculate the entire simulation at startup
    virtual void StartSimulation() {
        fCache.reserve(fSim->GetDefaultEndTime()+1);
        GetState(fSim->GetDefaultEndTime());
    }
    virtual void ReadData()     { }
    virtual bool finished()     { return true; }
    virtual int GetDescriptor() { return -1; }
    
    virtual double GetTimestep()
        { return fSim->GetTimestep(); }    // in units of seconds
    virtual int GetDefaultEndTime()
        { return fSim->GetDefaultEndTime(); } // in units of TIMESTEPs

    virtual const char* GetTitle()
        { return fSim->GetTitle(); }
    
    // Interface for draw modes
    virtual int GetNDrawModes() const { return fSim->GetNDrawModes(); }
    virtual const char* GetDrawModeName(int id) { return fSim->GetDrawModeName(id); }
    
    // Interface for data display
    virtual int GetNDataCh() const { return fSim->GetNDataCh(); }
    virtual const char* GetChName(int ch) const { return fSim->GetChName(ch); }
    
  private:
    Sim_T* fSim;
    
    std::vector<State_T> fCache;
};
#endif
