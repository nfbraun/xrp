#include "SimulationViewer.h"
#include "Acrobot.h"

int main(int argc, char** argv)
{
    Acrobot sim;
    
    return ShowSimulation(&sim, argc, argv);
}
