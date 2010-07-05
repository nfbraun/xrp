#include "SimulationViewer.h"
#include "Hobbelen.h"

int main(int argc, char** argv)
{
    Hobbelen sim;
    
    return ShowSimulation(&sim, argc, argv);
}
