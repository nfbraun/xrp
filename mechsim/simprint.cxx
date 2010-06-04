#include <iostream>
#include <cmath>
#include "McGeer.h"

int main()
{
    McGeer sim;
    
    for(int t=0; t<sim.GetDefaultEndTime(); t++) {
        const MGState* state = sim.GetState(t);
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << state->fLLeg.thighAng() << " " << state->fLLeg.shankAng() << " ";
        std::cout << state->fRLeg.thighAng() << " " << state->fRLeg.shankAng();
        std::cout << std::endl;
    }
    
    return 0;
}
