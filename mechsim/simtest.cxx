#include <QWidget>
#include <QSlider>
#include <QHBoxLayout>
#include <QGLWidget>
#include <QApplication>

#include "Test.h"
#include "GLWidget.h"
#include "SimulationViewer.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    TestSim sim;
    SimulationViewer v(&sim);
    
    v.show();
    
    return app.exec();
}
