#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "ShadowTest.h"

int main(int argc, char** argv)
{
    ShadowTest sim;
    SyncSimRunner<ShadowTest, STState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
