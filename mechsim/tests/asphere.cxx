#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "AsymSphere.h"

int main(int argc, char** argv)
{
    AsymSphere sim;
    SyncSimRunner<AsymSphere, ASState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
