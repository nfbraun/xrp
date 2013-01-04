#include "StateMachine.h"

/**
  This method is used to determine if, based on the parameters passed in and the type of state this is,
  the current state in the controller FSM needs to be transitioned from.
*/
bool StateMachine::needTransition(double phi, double swingFootVerticalForce, double stanceFootVerticalForce)
{
    //if we are to allow a transition on foot contact, we need to take care of the possibility that it
    //will occur early. In this case, we may still want to switch. If phi is at least this, then it is assumed
    //that we can transition;
    const double minPhiBeforeTransitionOnFootContact = 0.5;
    //also, in order to make sure that we don't transition tooooo early, we expect a minimum force applied on the swing foot before
    //it should register as a contact
    const double minSwingFootForceForContact = 20.0;
    
    //transition if we have a meaningful foot contact, and if it does not happen too early on...
    if ((phi > minPhiBeforeTransitionOnFootContact && swingFootVerticalForce > minSwingFootForceForContact) || phi >= 1) {
        return true;
    } else {
        return false;
    }
}
