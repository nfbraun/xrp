#pragma once

#include <cartwheel/SimGlobals.h>
#include <cartwheel/RobotInfo.h>
#include <cartwheel/ContactInfo.h>

class StateMachine {
  public:
    StateMachine(int startStance) : fStance(startStance), fPhi(0.) {}
    
    int stance() const { return fStance; }
    double phi() const { return fPhi; }
    
    bool advanceInTime(double dt, double stepTime, const RobotInfo& rinfo, const ContactInfo& cfs);
    
  private:
    // this value indicates which side is the stance side. 
    int fStance;
    
    //the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
    double fPhi;
    
    void setStance(int newStance) { fStance = newStance; }
    void toggleStance() {
        if(fStance == LEFT_STANCE) setStance(RIGHT_STANCE);
        else setStance(LEFT_STANCE);
    }
    
    bool needTransition(double phi, double swingFootVerticalForce, double stanceFootVerticalForce);
};
