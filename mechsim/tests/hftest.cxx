#include "SimulationViewer.h"
#include "HFTest.h"

int main(int argc, char** argv)
{
    HFTest sim;
    
    return ShowSimulation(&sim, argc, argv);
}
