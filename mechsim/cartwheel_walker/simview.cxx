#include <SimulationViewer.h>
#include <Cartwheel.h>
#include <AsyncSimRunner.h>

int main(int argc, char** argv)
{
    Cartwheel sim;
    AsyncSimRunner<Cartwheel, CartState> runner(&sim);
    
    return ShowSimulation(&runner, argc, argv);
}
