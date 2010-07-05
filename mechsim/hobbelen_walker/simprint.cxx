#include <iostream>
#include <cmath>
#include "Hobbelen.h"

int main()
{
    Hobbelen sim;
    
    for(int t=0; t<4*sim.GetDefaultEndTime(); t++) {
        const HobState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << std::endl;
    }
    
    return 0;
}

