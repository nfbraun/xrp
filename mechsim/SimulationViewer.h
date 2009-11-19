#ifndef __SIMULATIONVIEWER_H__
#define __SIMULATIONVIEWER_H__

#include <QWidget>
#include <QKeyEvent>
#include <QGridLayout>
#include <QString>
#include <QLineEdit>
#include <QPushButton>
#include <QValidator>
#include "GLWidget.h"

class QSlider;

class SimulationViewer : public QWidget
{
    Q_OBJECT
    
    public:
        SimulationViewer(Simulation* sim);
        
    public slots:
        void toggleSimulationRunning();
        void updateCamInfo();
        void setCamPos();
        void setCamTheta();
        void setCamPhi();

    protected:
        void keyPressEvent(QKeyEvent* ev);
        
    private:
        QLineEdit* makeInput(QGridLayout* l, const QString& text, int row);
    
        GLWidget *fGLWidget;
        QSlider *fTimeSlide;
        QPushButton *fStartPauseButton, *fHomeButton;
        QLineEdit *feX, *feY, *feZ, *feP, *feT;
        
        static const char START[];
        static const char PAUSE[];
};


#endif
