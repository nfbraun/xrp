#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QTimer>
#include <QGLWidget>
#include "Simulation.h"
#include "Vector.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
    
  public:
    GLWidget(Simulation* sim, QWidget *parent = 0);
    ~GLWidget();
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    bool isPaused() { return fPaused; }
    void start();
    void pause();
    
    static void drawSphere(double r, Vector3 p);
    static void drawTube(double r, Vector3 p1, Vector3 p2);
    
    static const double RAD_TO_DEG;
    static const double DEG_TO_RAD;
    
  public slots:
    void timestep();
    void setTime(int t);
    
  signals:
    void timeChanged(int t);
    
  protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    
    void mousePressEvent(QMouseEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    
    void camPosChanged();
    
    void drawStatusText();
        
    QPoint fLastPos;
    QTimer* fTimer;
    Vector3 fCamPos;
    int fPhi, fTheta;  // in units of ANG_STEPs
    int fT;  // in units of Simulation::TIMESTEPs
    bool fPaused;
    
    static const double ANG_STEP;
    
    Simulation* fSimulation;
};

#endif
