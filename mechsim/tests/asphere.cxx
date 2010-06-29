#include "SimulationViewer.h"
#include "AsymSphere.h"

int main(int argc, char** argv)
{
    AsymSphere sim;
    
    return ShowSimulation(&sim, argc, argv);
}
