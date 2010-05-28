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
    GLWidget(Simulation* sim, QWidget *parent = 0, bool paused = false);
    ~GLWidget();
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    inline double getCamX()     { return fX; }
    inline double getCamY()     { return fY; }
    inline Vector3 getCamPos() {
        if(fTrackObject)
            return fCenter + Vector3::Spherical(fDist, getCamTheta(),
                                                getCamPhi());
        else
            return fCamPos;
    }
    
    inline double getCamDist()  {
        if(fTrackObject)
            return fDist;
        else
            return (fCamPos-getCenter()).r();
    }
    
    inline double getCamTheta()     { return fTheta*ANG_UNIT; }
    inline double getCamPhi()       { return fPhi*ANG_UNIT; }
    inline double getCamRoll()      { return fRoll; }
    
    inline double getCamThetaDeg()  { return getCamTheta()*RAD_TO_DEG; }
    inline double getCamPhiDeg()    { return getCamPhi()*RAD_TO_DEG; }
    inline double getCamRollDeg()   { return getCamRoll()*RAD_TO_DEG; }
    
    inline Vector3 getCenterOffset()  { return fCenterOffset; }
    inline Vector3 getCenter()        { return fCenter; }
    
    inline Vector3 getCamDir()
        { return Vector3::Spherical(1., ANG_UNIT*fTheta, ANG_UNIT*fPhi); }
    inline Vector3 getHorizDir()
        { return Vector3::Spherical(1., M_PI/2., ANG_UNIT*(fPhi - PI_UNITS/2)); }
    inline Vector3 getVertDir() {
        if(fTheta < PI_UNITS/2) {
            return Vector3::Spherical(1.,
                                      ANG_UNIT*(PI_UNITS/2 - fTheta),
                                      ANG_UNIT*(fPhi+PI_UNITS));
        } else {
            return Vector3::Spherical(1.,
                                      ANG_UNIT*(fTheta - PI_UNITS/2),
                                      ANG_UNIT*fPhi);
        }
    }
    
    inline Vector3 getUpDir() {
        if(fEnableRoll) {
            return getVertDir()*cos(fRoll) + getHorizDir()*sin(fRoll);
        } else {
            return getVertDir();
        }
    }
    
    void setCamPos(Vector3 pos);
    void setCamDist(double dist);
    void setCamThetaDeg(double theta);
    void setCamPhiDeg(double phi);
    void setCamRollDeg(double roll);
    void setCenterOffset(Vector3 offset);
    void setTrackObject(bool track);
    void setEnableRoll(bool enable);
    
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
    
    void rotateMotion(QMouseEvent* ev);
    double rollAngle(int x, int y);
    void rollMotion(QMouseEvent* ev);
    void panMotion(QMouseEvent* ev);
    void zoomMotion(QMouseEvent* ev);
    
    void mousePressEvent(QMouseEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    void wheelEvent(QWheelEvent* ev);
    
    void _updateCam();
    void updateCam();
    void normalizeAngles();
    
    void drawStatusText();
    
    enum MotionMode { MODE_NONE, MODE_ROTATE, MODE_ZOOM, MODE_ROLL, MODE_PAN };
    
    MotionMode fMotionMode;
    Vector3 fLastVec;
    double fLastRoll;
    QPoint fLastPos;
    QTimer* fTimer;
    Vector3 fCamPos;
    int fTheta, fPhi;
    double fRoll;
    double fX, fY, fDist;
    Vector3 fCenter, fCenterOffset;
    int fCenterX, fCenterY;
    int fT;  // in units of Simulation::TIMESTEPs
    bool fPaused;
    bool fTrackObject, fEnableRoll;
    
    GLdouble fGLModelview[16];
    GLdouble fGLProjection[16];
    GLint fGLViewport[4];
    
    static const int PI_UNITS;
    static const double ANG_UNIT;
    static const int ANG_STEP_TRACKMODE, ANG_STEP_WALKMODE;
    static const double DIST_STEP;
    static const double SHIFT_STEP;
    static const double WALK_STEP;
    static const int DIST_WHEEL_CORRECTION;
    
    static const double AXIS_LENGTH;
    static const float X_AXIS_COLOR[];
    static const float Y_AXIS_COLOR[];
    static const float Z_AXIS_COLOR[];
    
    Simulation* fSimulation;
};

#endif
