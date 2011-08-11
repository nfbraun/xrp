#ifndef MSIM_CUBE_H
#define MSIM_CUBE_H

#include "SyncSimulation.h"
#include <GL/gl.h>

class Cube;

class CubeState: public SimulationState {
  public:
    double fT;
    void Draw(int) const;
    
    Vector3 GetCenter() const
     { return Vector3(0.,0.,1.); }
    
    Cube* fParent;
};

class Cube: public SyncSimulation<CubeState> {
  friend class CubeState;
  public:
    Cube();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60. * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    inline void Advance() { fCurStep++; };
    CubeState GetCurrentState();
    
    int fCurStep;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
};

#endif
