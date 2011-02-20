#include <iostream>
#include <cmath>
#include "McGeer.h"

int main()
{
    McGeer sim;
    
    std::cout << "#:1:Inner thigh angle" << std::endl;
    std::cout << "#:2:Inner shank angle" << std::endl;
    std::cout << "#:3:Outer thigh angle" << std::endl;
    std::cout << "#:4:Outer shank angle" << std::endl;
    std::cout << "#:5:Inner thigh speed" << std::endl;
    std::cout << "#:6:Inner shank speed" << std::endl;
    std::cout << "#:7:Outer thigh speed" << std::endl;
    std::cout << "#:8:Outer shank speed" << std::endl;
    std::cout << "#:9:Inner foot clearance" << std::endl;
    std::cout << "#:10:Outer foot clearance" << std::endl;
    
    for(int t=0; t<sim.GetDefaultEndTime(); t++) {
        const MGState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << state->fILeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fILeg.shankAng() - McGeer::EPS_T << " ";
        std::cout << state->fOLeg.thighAng() - McGeer::EPS_T << " ";
        std::cout << state->fOLeg.shankAng() - McGeer::EPS_T << " ";
        std::cout << state->fILeg.omegaT() << " ";
        std::cout << state->fILeg.omegaS() << " ";
        std::cout << state->fOLeg.omegaT() << " ";
        std::cout << state->fOLeg.omegaS() << " ";
        std::cout << state->fILeg.footClearance() << " ";
        std::cout << state->fOLeg.footClearance() << " ";
        // std::cout << state->fILeg.shankCoG().y() << " ";
        // std::cout << state->fOLeg.shankCoG().y() << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}

