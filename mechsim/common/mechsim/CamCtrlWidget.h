#ifndef MSIM_CAMCTRLWIDGET_H
#define MSIM_CAMCTRLWIDGET_H

#include <QString>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QGridLayout>
#include "GLWidget.h"

class CamCtrlWidget: public QWidget
{
    Q_OBJECT
    
    public:
        CamCtrlWidget(QWidget* parent, GLWidget* glwidget);
    
    public slots:
        void updateCamInfo();
        void setCamPos();
        void setCamDist();
        void setCamTheta();
        void setCamPhi();
        void setCamRoll();
        void setCenterOffset();
        void setTrackObject(int state);
        void setEnableRoll(int state);
    
    private:
        QLineEdit* makeInput(QGridLayout* l, const QString& text, int row, const char* slot = NULL);
        
        GLWidget* fGLWidget;
        QPushButton *fHomeButton;
        QLineEdit *feX, *feY, *feZ;
        QLineEdit *fePhi, *feTheta, *feRoll, *feDist;
        QLineEdit *fecx, *fecy, *fecz;
};

#endif
