#include "SimulationViewer.h"
#include <QSlider>
#include <QVBoxLayout>

SimulationViewer::SimulationViewer(Simulation* sim)
{
    fGLWidget = new GLWidget(sim);
    fTimeSlide = new QSlider(Qt::Horizontal);
    fTimeSlide->setRange(0, sim->GetDefaultEndTime());
    
    connect(fTimeSlide, SIGNAL(valueChanged(int)), fGLWidget, SLOT(setTime(int)));
    connect(fGLWidget, SIGNAL(timeChanged(int)), fTimeSlide, SLOT(setValue(int)));
    
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(fGLWidget);
    layout->addWidget(fTimeSlide);
    setLayout(layout);
    
    setWindowTitle(sim->GetTitle());
}

void SimulationViewer::keyPressEvent(QKeyEvent* ev)
{
    if(ev->key() == Qt::Key_Space) {
        if(fGLWidget->isPaused())
            fGLWidget->start();
        else
            fGLWidget->pause();
    } else {
        ev->ignore();
    }
}
