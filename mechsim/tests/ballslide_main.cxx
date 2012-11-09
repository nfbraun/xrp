#include "SimulationViewer.h"
#include "AsyncSimRunner.h"
#include "BallSlide.h"

int main(int argc, char** argv)
{
    BallSlide sim;
    AsyncSimRunner<BallSlide, BSState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
