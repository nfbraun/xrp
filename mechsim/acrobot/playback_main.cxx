#include "SimulationViewer.h"
#include "SyncSimRunner.h"
#include "Playback.h"

int main(int argc, char** argv)
{
    Playback playback;
    SyncSimRunner<Playback, AcroState> runner(&playback);
    
    return ShowSimulation(&runner, argc, argv);
}
