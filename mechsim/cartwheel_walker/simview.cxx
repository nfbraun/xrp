#include "SimulationViewer.h"
#include "Cartwheel.h"

int main(int argc, char** argv)
{
    Cartwheel sim;
    
    return ShowSimulation(&sim, argc, argv);
}
