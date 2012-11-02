#include "SimulationViewer.h"
#include "AsyncSimRunner.h"
#include "HFTest.h"

int main(int argc, char** argv)
{
    HFTest sim;
    AsyncSimRunner<HFTest, HFState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
