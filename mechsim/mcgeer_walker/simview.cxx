#include "SimulationViewer.h"
#include "McGeer.h"

int main(int argc, char** argv)
{
    McGeer sim;
    
    return ShowSimulation(&sim, argc, argv);
}
