#include <iostream>
#include <cmath>
#include "McGeer.h"

int main()
{
    McGeer sim;
    
    for(int t=0; t<4*sim.GetDefaultEndTime(); t++) {
        const MGState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << state->fLLeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fLLeg.shankAng() - McGeer::EPS_T << " ";
        std::cout << state->fRLeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fRLeg.shankAng() - McGeer::EPS_T << " ";
        /* std::cout << state->fLLeg.omegaT() << " ";
        std::cout << state->fLLeg.omegaS() << " ";
        std::cout << state->fRLeg.omegaT() << " ";
        std::cout << state->fRLeg.omegaS() << " "; */
        std::cout << state->fLLeg.footClearance() << " ";
        //std::cout << state->fRLeg.footClearance() << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}

