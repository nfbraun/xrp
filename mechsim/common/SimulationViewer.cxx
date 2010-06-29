#include "SimulationViewer.h"
#include "SimulationWidget.h"
#include <QApplication>

int ShowSimulation(Simulation* sim, int argc, char** argv)
{
    QApplication app(argc, argv);
    SimulationWidget v(sim);
    
    v.show();
    
    return app.exec();
}

