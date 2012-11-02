#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "McGeer.h"

int main(int argc, char** argv)
{
    McGeer sim;
    SyncSimRunner<McGeer, MGState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
