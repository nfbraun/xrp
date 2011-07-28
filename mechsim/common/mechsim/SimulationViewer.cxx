#include "SimulationViewer.h"
#include "SimulationWidget.h"
#include <QApplication>
#include <iostream>

int ShowSimulation(Simulation* sim, int argc, char** argv)
{
    QApplication app(argc, argv);
    
    sim->StartSimulation();
    while(sim->GetState(0) == 0)
        sim->ReadData();
    
    SimulationWidget v(sim);
    v.show();
    
    return app.exec();
}

