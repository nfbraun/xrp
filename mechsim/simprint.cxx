#include <iostream>
#include <cmath>
#include "Test.h"

int main()
{
    TestSim sim;
    
    for(int t=0; t<sim.GetDefaultEndTime(); t++) {
        const TestState* state = sim.GetState(t);
        // for definition compatible with lagrange.c
        const double phi = -(state->fPos.x() + .4)/TestSim::R + M_PI/2.;
        std::cout << t*sim.GetTimestep() << " ";
        std::cout << phi;
        std::cout << std::endl;
    }
    
    return 0;
}
