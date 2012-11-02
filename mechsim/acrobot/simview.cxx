#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "Acrobot.h"

int main(int argc, char** argv)
{
    Acrobot sim;
    SyncSimRunner<Acrobot, AcroState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
