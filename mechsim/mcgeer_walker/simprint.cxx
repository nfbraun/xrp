#include <iostream>
#include <cmath>
#include "McGeer.h"

int main()
{
    McGeer sim;
    
    for(int t=0; t<4*sim.GetDefaultEndTime(); t++) {
        const MGState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << state->fILeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fILeg.shankAng() - McGeer::EPS_T << " ";
        std::cout << state->fOLeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fOLeg.shankAng() - McGeer::EPS_T << " ";
        /* std::cout << state->fILeg.omegaT() << " ";
        std::cout << state->fILeg.omegaS() << " ";
        std::cout << state->fOLeg.omegaT() << " ";
        std::cout << state->fOLeg.omegaS() << " "; */
        std::cout << state->fILeg.footClearance() << " ";
        //std::cout << state->fOLeg.footClearance() << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}

