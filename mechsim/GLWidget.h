#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QTimer>
#include <QGLWidget>
#include "Simulation.h"
#include "Vector.h"
#include "Rotation.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
    
  public:
    GLWidget(Simulation* sim, QWidget *parent = 0);
    ~GLWidget();
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    inline double getCamX()     { return fX; }
    inline double getCamY()     { return fZ; }
    inline double getCamDist()  { return exp(fDist * DIST_STEP); }
    inline Vector3 getCenter()  { return fCenterOffset; }
    
    void setCamX(double x);
    void setCamY(double y);
    void setCamDist(double dist);
    void setCamRotation(const Rotation& r);
    void rotateCam(const Rotation& r);
    
    bool isPaused() { return fPaused; }
    void start();
    void pause();
    
    static void drawSphere(double r, Vector3 p);
    static void drawDiscSegment(double r, double h, double alpha);
    static void drawCheckerboardFloor();
    static void drawTube(double r, Vector3 p1, Vector3 p2);
    static void drawODEBox(dGeomID id, double lx, double ly, double lz);
    static void drawBox(Vector3 p1, Vector3 p2);
    static void drawUnitCube();
    void drawCenter();
    
    static const double RAD_TO_DEG;
    static const double DEG_TO_RAD;
    static const int DEFAULT_CAM_DIST;
    
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
    
    Vector3 trackballVector(int px, int py);
    void trackballMotion(QMouseEvent* ev);
    double rollAngle(int x, int y);
    void rollMotion(QMouseEvent* ev);
    void panMotion(QMouseEvent* ev);
    void zoomMotion(QMouseEvent* ev);
    
    void mousePressEvent(QMouseEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    void wheelEvent(QWheelEvent* ev);
    
    void _updateCam();
    void updateCam();
    
    void drawStatusText();
    
    enum MotionMode { MODE_NONE, MODE_ROTATE, MODE_ZOOM, MODE_ROLL, MODE_PAN };
    
    MotionMode fMotionMode;
    Vector3 fLastVec;
    double fLastRoll;
    QPoint fLastPos;
    QTimer* fTimer;
    double fX, fZ;
    double fDist;
    Rotation fRotation;
    Vector3 fCenter, fCenterOffset;
    int fCenterX, fCenterY;
    int fT;  // in units of Simulation::TIMESTEPs
    bool fPaused;
    bool fTrackObject;
    
    GLdouble fGLModelview[16];
    GLdouble fGLProjection[16];
    GLint fGLViewport[4];
    
    static const int ANG_STEPS_PER_PI;
    static const double ANG_STEP;
    static const double DIST_STEP;
    static const double SHIFT_STEP;
    static const int DIST_WHEEL_CORRECTION;
    
    static const double AXIS_LENGTH;
    static const float X_AXIS_COLOR[];
    static const float Y_AXIS_COLOR[];
    static const float Z_AXIS_COLOR[];
    
    Simulation* fSimulation;
};

#endif
