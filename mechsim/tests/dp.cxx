#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "DoublePendulum.h"

int main(int argc, char** argv)
{
    DoublePendulum sim;
    SyncSimRunner<DoublePendulum, DPState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
