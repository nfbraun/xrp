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
    
    inline Vector3 getCamPos() { return fCamPos; }
    inline double getCamPhi() { return fPhi * (ANG_STEP * RAD_TO_DEG); }
    inline double getCamTheta() { return fTheta * (ANG_STEP * RAD_TO_DEG); }
    
    void setCamPos(double x, double y, double z);
    void setCamPhi(double phi);
    void setCamTheta(double theta);
    
    bool isPaused() { return fPaused; }
    void start();
    void pause();
    
    static void drawSphere(double r, Vector3 p);
    static void drawCheckerboardFloor();
    static void drawTube(double r, Vector3 p1, Vector3 p2);
    static void drawODEBox(dGeomID id, double lx, double ly, double lz);
    static void drawBox(Vector3 p1, Vector3 p2);
    static void drawUnitCube();
    
    static const double RAD_TO_DEG;
    static const double DEG_TO_RAD;
    static const Vector3 DEFAULT_CAM_POS;
    
  public slots:
    void timestep();
    void setTime(int t);
    void resetCamPos();
    
  signals:
    void timeChanged(int t);
    void camChanged();
    
  protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void normalizeAngles();
    
    void mousePressEvent(QMouseEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    void wheelEvent(QWheelEvent* ev);
    
    void _updateCam();
    void updateCam();
    
    void drawStatusText();
        
    QPoint fLastPos;
    QTimer* fTimer;
    Vector3 fCamPos;
    int fPhi, fTheta;  // in units of ANG_STEPs
    int fT;  // in units of Simulation::TIMESTEPs
    bool fPaused;
    
    static const int ANG_STEPS_PER_PI;
    static const double ANG_STEP;
    static const double WHEEL_STEP;
    
    Simulation* fSimulation;
};

#endif
