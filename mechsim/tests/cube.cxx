#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "Cube.h"

int main(int argc, char** argv)
{
    Cube sim;
    SyncSimRunner<Cube, CubeState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
