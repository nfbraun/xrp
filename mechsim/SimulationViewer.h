#ifndef __SIMULATIONVIEWER_H__
#define __SIMULATIONVIEWER_H__

#include <QWidget>
#include <QKeyEvent>
#include "GLWidget.h"

class QSlider;

class SimulationViewer : public QWidget
{
    Q_OBJECT
    
    public:
        SimulationViewer(Simulation* sim);

    protected:
        void keyPressEvent(QKeyEvent* ev);
        
    private:
        GLWidget *fGLWidget;
        QSlider *fTimeSlide;
};


#endif
