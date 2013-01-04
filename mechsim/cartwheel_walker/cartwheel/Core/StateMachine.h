#pragma once

#include "SimBiConState.h"

class StateMachine {
  public:
    //this value indicates which side is the stance side. 
    int stance;
    
    //the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
    double phi;
    
    SimBiConState state;
    
    void setStepTime(double t) {
        state.setStateTime(t);
    }
    
    double getStepTime() {
        return state.getStateTime();
    }
    
    bool needTransition(double phi, double swingFootVerticalForce, double stanceFootVerticalForce);
};
