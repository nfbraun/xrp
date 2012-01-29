#include "SimulationViewer.h"
#include "Playback.h"

int main(int argc, char** argv)
{
    Playback playback;
    
    return ShowSimulation(&playback, argc, argv);
}
