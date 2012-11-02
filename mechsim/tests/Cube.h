#ifndef MSIM_CUBE_H
#define MSIM_CUBE_H

#include "Simulation.h"
#include <Eigen/Dense>
#include <GL/gl.h>

class Cube;

class CubeState: public SimulationState {
  public:
    double fT;
    void Draw(int) const;
    
    Eigen::Vector3d GetCenter() const
     { return Eigen::Vector3d(0.,0.,1.); }
    
    Cube* fParent;
};

class Cube: public Simulation {
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
