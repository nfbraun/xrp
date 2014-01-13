#ifndef MSIM_SHADOWTEST_H
#define MSIM_SHADOWTEST_H

#include "Simulation.h"
#include "Shadow.h"
#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glu.h>

class ShadowTest;

class STState: public SimulationState {
  public:
    double fT;
    void Draw(int) const;
    
    Eigen::Vector3d GetCenter() const
     { return Eigen::Vector3d(0.,0.,1.); }
    
    ShadowTest* fParent;
};

class ShadowTest: public Simulation {
  friend class STState;
  public:
    ShadowTest();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60. * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    inline void Advance() { fCurStep++; };
    STState GetCurrentState();
    
    int fCurStep;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
};

#endif
