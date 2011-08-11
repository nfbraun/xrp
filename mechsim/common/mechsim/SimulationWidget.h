#ifndef MSIM_SIMULATIONWIDGET_H
#define MSIM_SIMULATIONWIDGET_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QGridLayout>
#include <QString>
#include <QLineEdit>
#include <QPushButton>
#include <QValidator>
#include <QSocketNotifier>
#include "GLWidget.h"
#include "Simulation.h"
#include "DataViewWidget.h"
#include "CamCtrlWidget.h"

class QSlider;

class SimulationWidget : public QMainWindow
{
    Q_OBJECT
    
    public:
        SimulationWidget(Simulation* sim);
        
    public slots:
        void toggleSimulationRunning();
        void setDrawMode(int mode);
        void simDataReady();
    
    signals:
        void simHasNewData();

    protected:
        void keyPressEvent(QKeyEvent* ev);
        
    private:
        void initDataView(Simulation* sim);
        
        GLWidget *fGLWidget;
        QSlider *fTimeSlide;
        DataViewWidget *fDataView;
        CamCtrlWidget *fCamCtrlWidget;
        QPushButton *fStartPauseButton;
        
        Simulation* fSimulation;
        QSocketNotifier* fSockNotifier;
        
        static const char START[];
        static const char PAUSE[];
};


#endif
