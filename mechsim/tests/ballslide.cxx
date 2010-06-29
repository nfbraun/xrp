#include "SimulationViewer.h"
#include "BallSlide.h"

int main(int argc, char** argv)
{
    BallSlide sim;
    
    return ShowSimulation(&sim, argc, argv);
}
