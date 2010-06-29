#include <QWidget>
#include <QSlider>
#include <QHBoxLayout>
#include <QGLWidget>
#include <QApplication>

#include "McGeer.h"
#include "GLWidget.h"
#include "SimulationViewer.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    McGeer sim;
    SimulationViewer v(&sim);
    
    v.show();
    
    return app.exec();
}
