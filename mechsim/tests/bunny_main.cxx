#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "Bunny.h"

int main(int argc, char** argv)
{
    Bunny sim;
    SyncSimRunner<Bunny, BunnyState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
