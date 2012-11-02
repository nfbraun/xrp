#ifndef MSIM_GLWIDGET_H
#define MSIM_GLWIDGET_H

#include <QTimer>
#include <QGLWidget>
#include "SimRunner.h"
#include "Spherical.h"
#include "GLHelper.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
    
  public:
    GLWidget(SimRunner* sim, QWidget *parent = 0, bool paused = false);
    ~GLWidget();
    
    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    
    inline double getCamX()     { return fX; }
    inline double getCamY()     { return fY; }
    inline Eigen::Vector3d getCamPos() {
        if(fTrackObject)
            return fCenter + Spherical(fDist, getCamTheta(),
                                       getCamPhi());
        else
            return fCamPos;
    }
    
    inline double getCamDist()  {
        if(fTrackObject)
            return fDist;
        else
            return Spherical(fCamPos-getCenter()).r();
    }
    
    inline double getCamTheta()     { return fTheta*ANG_UNIT; }
    inline double getCamPhi()       { return fPhi*ANG_UNIT; }
    inline double getCamRoll()      { return fRoll; }
    
    inline double getCamThetaDeg()  { return getCamTheta()*RAD_TO_DEG; }
    inline double getCamPhiDeg()    { return getCamPhi()*RAD_TO_DEG; }
    inline double getCamRollDeg()   { return getCamRoll()*RAD_TO_DEG; }
    
    inline Eigen::Vector3d getCenterOffset()  { return fCenterOffset; }
    inline Eigen::Vector3d getCenter()        { return fCenter; }
    
    inline Eigen::Vector3d getCamDir()
        { return Spherical(1., ANG_UNIT*fTheta, ANG_UNIT*fPhi); }
    inline Eigen::Vector3d getHorizDir()
        { return Spherical(1., M_PI/2., ANG_UNIT*(fPhi - PI_UNITS/2)); }
    inline Eigen::Vector3d getVertDir() {
        if(fTheta < PI_UNITS/2) {
            return Spherical(1.,
                             ANG_UNIT*(PI_UNITS/2 - fTheta),
                             ANG_UNIT*(fPhi+PI_UNITS));
        } else {
            return Spherical(1.,
                             ANG_UNIT*(fTheta - PI_UNITS/2),
                             ANG_UNIT*fPhi);
        }
    }
    
    inline Eigen::Vector3d getUpDir() {
        if(fEnableRoll) {
            return getVertDir()*cos(fRoll) + getHorizDir()*sin(fRoll);
        } else {
            return getVertDir();
        }
    }
    
    void setCamPos(Eigen::Vector3d pos);
    void setCamDist(double dist);
    void setCamThetaDeg(double theta);
    void setCamPhiDeg(double phi);
    void setCamRollDeg(double roll);
    void setCenterOffset(Eigen::Vector3d offset);
    void setTrackObject(bool track);
    void setEnableRoll(bool enable);
    
    bool isPaused() { return fPaused; }
    void start();
    void pause();
    
    void drawCenter();
    
    static const double RAD_TO_DEG;
    static const double DEG_TO_RAD;
    static const int DEFAULT_CAM_DIST;
    
  public slots:
    void timestep();
    void setTime(int t);
    void setDrawMode(int mode);
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
    Eigen::Vector3d fLastVec;
    double fLastRoll;
    QPoint fLastPos;
    QTimer* fTimer;
    Eigen::Vector3d fCamPos;
    int fTheta, fPhi;
    double fRoll;
    double fX, fY, fDist;
    Eigen::Vector3d fCenter, fCenterOffset;
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
    
    SimRunner* fSimRunner;
    int fDrawMode;
};

#endif
