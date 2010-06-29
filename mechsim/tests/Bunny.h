#ifndef __BUNNY_H__
#define __BUNNY_H__

#include "CachedSimulation.h"
#include <GL/gl.h>

class Bunny;

class BunnyState: public SimulationState {
  public:
    double fT;
    void Draw() const;
    
    // calculated with vtk
    Vector3 GetCenter() const
     { return 10.*Vector3(.2 * sin(fT / 8.), 0., 0.154/2.); }
    
    Bunny* fParent;
};

class Bunny: public CachedSimulation<BunnyState> {
  friend class BunnyState;
  public:
    Bunny();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60. * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    inline void Advance() { fCurStep++; };
    BunnyState GetCurrentState();
    
    int fCurStep;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
  private:
    GLuint GetBunnyList();
    GLuint fGLList;
};

#endif
