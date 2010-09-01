#ifndef __SIMULATIONWIDGET_H__
#define __SIMULATIONWIDGET_H__

#include <QWidget>
#include <QKeyEvent>
#include <QGridLayout>
#include <QString>
#include <QLineEdit>
#include <QPushButton>
#include <QValidator>
#include "GLWidget.h"

class QSlider;

class SimulationWidget : public QWidget
{
    Q_OBJECT
    
    public:
        SimulationWidget(Simulation* sim);
        
    public slots:
        void toggleSimulationRunning();
        void updateCamInfo();
        void setCamPos();
        void setCamDist();
        void setCamTheta();
        void setCamPhi();
        void setCamRoll();
        void setCenterOffset();
        void setTrackObject(int state);
        void setEnableRoll(int state);

    protected:
        void keyPressEvent(QKeyEvent* ev);
        
    private:
        QLineEdit* makeInput(QGridLayout* l, const QString& text, int row, const char* slot = NULL);
    
        GLWidget *fGLWidget;
        QSlider *fTimeSlide;
        QPushButton *fStartPauseButton, *fHomeButton;
        QLineEdit *feX, *feY, *feZ;
        QLineEdit *fePhi, *feTheta, *feRoll, *feDist;
        QLineEdit *fecx, *fecy, *fecz;
        
        static const char START[];
        static const char PAUSE[];
};


#endif
