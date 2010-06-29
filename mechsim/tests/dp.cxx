#include "SimulationViewer.h"
#include "DoublePendulum.h"

int main(int argc, char** argv)
{
    DoublePendulum sim;
    
    return ShowSimulation(&sim, argc, argv);
}
