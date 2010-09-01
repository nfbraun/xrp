#ifndef __CACHEDSIMULATION_H__
#define __CACHEDSIMULATION_H__

#include "Simulation.h"
#include <vector>

template<class State_T>
class CachedSimulation: public Simulation
{
  public:
    virtual const State_T* GetState(int t)  // t in units of TIMESTEPs
    {
        while((int)fCache.size() <= t) {
            fCache.push_back(GetCurrentState());
            Advance();
        }
        
        return &fCache[t];
    }
    
  protected:
    virtual void Advance() = 0;
    virtual State_T GetCurrentState() = 0;
    
  private:
    std::vector<State_T> fCache;
};
#endif
