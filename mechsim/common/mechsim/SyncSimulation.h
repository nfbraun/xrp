#ifndef MSIM_SYNCSIMULATION_H
#define MSIM_SYNCSIMULATION_H

#include "Simulation.h"
#include <vector>

// Interface for simulations that can be completed near-instantaneously
// at program startup
template<class State_T>
class SyncSimulation: public Simulation
{
  public:
    virtual const State_T* GetState(int t)  // t in units of TIMESTEPs
    {
        if(t < 0) return 0;
        
        while((int)fCache.size() <= t) {
            fCache.push_back(GetCurrentState());
            Advance();
        }
        
        return &fCache[t];
    }
    
    // Calculate the entire simulation at startup
    virtual void StartSimulation() {
        fCache.reserve(GetDefaultEndTime()+1);
        GetState(GetDefaultEndTime());
    }
    virtual void ReadData()     { }
    virtual bool finished()     { return true; }
    virtual int GetDescriptor() { return -1; }
    
  protected:
    virtual void Advance() = 0;
    virtual State_T GetCurrentState() = 0;
    
  private:
    std::vector<State_T> fCache;
};
#endif
