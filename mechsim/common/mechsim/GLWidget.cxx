#include "GLWidget.h"
#include <QtOpenGL>
#include <QString>
#include <QFont>
#include <QFontMetrics>
#include <GL/glu.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>

const double GLWidget::RAD_TO_DEG = 180. / M_PI;
const double GLWidget::DEG_TO_RAD = M_PI / 180.;
const int GLWidget::PI_UNITS = 900;
const double GLWidget::ANG_UNIT = M_PI / PI_UNITS;
const int GLWidget::ANG_STEP_TRACKMODE = 10;
const int GLWidget::ANG_STEP_WALKMODE = 5;
const double GLWidget::DIST_STEP = 0.01;
const int GLWidget::DIST_WHEEL_CORRECTION = 10;
const double GLWidget::SHIFT_STEP = 0.2;
const double GLWidget::WALK_STEP = 0.5;

const double GLWidget::AXIS_LENGTH = 1.0;
const float GLWidget::X_AXIS_COLOR[] = { 1.0, 0.0, 0.0 };
const float GLWidget::Y_AXIS_COLOR[] = { 1.0, 1.0, 0.0 };
const float GLWidget::Z_AXIS_COLOR[] = { 0.0, 1.0, 0.0 };

const int GLWidget::DEFAULT_CAM_DIST = 5.;

GLWidget::GLWidget(Simulation* sim, QWidget* parent, bool paused)
    : QGLWidget(parent),
      fCamPos(0., 0., 0.),
      fTheta(PI_UNITS/2), fPhi(3*PI_UNITS/2), fRoll(0.),
      fX(0.), fY(0.), fDist(DEFAULT_CAM_DIST),
      fCenterOffset(Vector3::Null),
      fT(0),
      fPaused(paused),
      fTrackObject(true), fEnableRoll(false),
      fSimulation(sim), fDrawMode(0)
{
    fTimer = new QTimer(this);
    fTimer->setInterval(sim->GetTimestep()*1000.);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(timestep()));
    
    if(!fPaused)
        fTimer->start();
    
    fCenter = fSimulation->GetState(fT)->GetCenter() + fCenterOffset;
}

GLWidget::~GLWidget()
{
    makeCurrent();
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(100, 100);
}

QSize GLWidget::sizeHint() const
{
    return QSize(640, 480);
}

void GLWidget::setCamPos(Vector3 pos)
{
    if(fTrackObject) {
        Vector3 d = pos - fCenter;
        fDist = d.r();
        fPhi =   (int) ceil((d.phi()/ANG_UNIT)-0.5);
        fTheta = (int) ceil((d.theta()/ANG_UNIT)-0.5);
    } else {
        fCamPos = pos;
    }
    updateCam();
}

void GLWidget::setCamDist(double d)
{
    fDist = d;
    updateCam();
}

void GLWidget::setCamThetaDeg(double theta)
{
    fTheta = (int) ceil((theta * DEG_TO_RAD / ANG_UNIT) - 0.5);
    normalizeAngles();
    updateCam();
}

void GLWidget::setCamPhiDeg(double phi)
{
    fPhi = (int) ceil((phi * DEG_TO_RAD / ANG_UNIT) - 0.5);
    normalizeAngles();
    updateCam();
}

void GLWidget::setCamRollDeg(double roll)
{
    roll = fmod(roll, 360.);
    if(roll < 0.) roll += 360.;
    fRoll = roll * DEG_TO_RAD;
    updateCam();
}

void GLWidget::setCenterOffset(Vector3 offset)
{
    fCenterOffset = offset;
    fCenter = fSimulation->GetState(fT)->GetCenter() + fCenterOffset;
    updateCam();
}

void GLWidget::setTrackObject(bool track)
{
    fTrackObject = track;
    
    if(fTrackObject) {
        Vector3 d = fCamPos - fCenter;
        fDist = d.r();
        fPhi =   (int) ceil((d.phi()/ANG_UNIT)-0.5);
        fTheta = (int) ceil((d.theta()/ANG_UNIT)-0.5);
        fX = 0.;
        fY = 0.;
    } else {
        fCamPos = fCenter + fDist*getCamDir() + fX*getHorizDir() - fY*getVertDir();
    }
    updateCam();
}

void GLWidget::setEnableRoll(bool enable)
{
    fEnableRoll = enable;
    updateCam();
}

void GLWidget::resetCamPos()
{
    fDist = DEFAULT_CAM_DIST;
    fTheta = PI_UNITS/2;
    fPhi = 0;
    fRoll = 0.;
    fX = 0.;
    fY = 0.;
    fCamPos = fCenter + fDist*getCamDir();
    
    updateCam();
}

void GLWidget::timestep()
{
    ++fT;
    if(fT >= fSimulation->GetDefaultEndTime())
        pause();
    else
        updateGL();
    emit timeChanged(fT);
}

void GLWidget::setTime(int t)
{
    if(t < 0) t = 0;
    if(fT == t) return;
    fT = t;
    updateGL();
    emit timeChanged(fT);
}

void GLWidget::setDrawMode(int mode)
{
    if(mode == fDrawMode) return;
    fDrawMode = mode;
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent* ev)
{
    fDist *= exp(-ev->delta() * DIST_STEP / DIST_WHEEL_CORRECTION);
    
    updateCam();
    emit camChanged();
    
    ev->accept();
}

void GLWidget::rotateMotion(QMouseEvent* ev)
{
    int dx = ev->x() - fLastPos.x();
    int dy = ev->y() - fLastPos.y();
    
    if(fTrackObject) {
        fPhi -= dx*ANG_STEP_TRACKMODE;
        fTheta -= dy*ANG_STEP_TRACKMODE;
    } else {
        fPhi += dx*ANG_STEP_WALKMODE;
        fTheta += dy*ANG_STEP_WALKMODE;
    }
    normalizeAngles();
    
    fLastPos = ev->pos();
}

double GLWidget::rollAngle(int x, int y)
{
    // Ensure that x and y can never be 0
    x = 2*(x - fCenterX) + 1;
    y = 2*(y - fCenterY) + 1;
    
    return atan2(y, x);
}

void GLWidget::rollMotion(QMouseEvent* ev)
{
    double roll2 = rollAngle(ev->x(), ev->y());
    
    fRoll += roll2 - fLastRoll;
    if(fRoll < 0) fRoll += 2.*M_PI;
    if(fRoll > 2.*M_PI) fRoll -= 2.*M_PI;
    
    fLastRoll = roll2;
}

void GLWidget::panMotion(QMouseEvent* ev)
{
    if(fTrackObject) {
        const GLdouble unit[16] = { 1., 0., 0., 0.,
                                    0., 1., 0., 0.,
                                    0., 0., 1., 0.,
                                    0., 0., 0., 1. };
        GLdouble wx, wy, wz;
        gluProject(0., 0., fDist,
                   unit, fGLProjection, fGLViewport,
                   &wx, &wy, &wz);
        
        GLdouble x1, y1, z1, x2, y2, z2;
        gluUnProject(fLastPos.x(), -fLastPos.y(), wz,
                     unit, fGLProjection, fGLViewport,
                     &x1, &y1, &z1);
        gluUnProject(ev->x(), -ev->y(), wz,
                     unit, fGLProjection, fGLViewport,
                     &x2, &y2, &z2);
        
        fX += x1 - x2;
        fY += y1 - y2;
    } else {
        fCamPos += (ev->x() - fLastPos.x()) * SHIFT_STEP * getHorizDir();
        fCamPos += (ev->y() - fLastPos.y()) * SHIFT_STEP * getVertDir();
    }
    
    fLastPos = ev->pos();
}

void GLWidget::zoomMotion(QMouseEvent* ev)
{
    int dist = ev->y() - fLastPos.y();
    if(fTrackObject) {
        fDist *= exp(-dist * DIST_STEP);
    } else {
        fCamPos += -dist * WALK_STEP * Vector3::Spherical(1., getCamTheta(),
                                                          getCamPhi());
    }
    fLastPos = ev->pos();
}

void GLWidget::mousePressEvent(QMouseEvent* ev)
{
    fMotionMode = MODE_NONE;
    
    switch(ev->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier)) {
        case Qt::NoModifier:
            if(ev->button() == Qt::LeftButton)
                fMotionMode = MODE_ROTATE;
            else if(ev->button() == Qt::RightButton)
                fMotionMode = MODE_ZOOM;
        break;
        case Qt::ShiftModifier:
            if(ev->button() == Qt::LeftButton && fEnableRoll)
                fMotionMode = MODE_ROLL;
            else if(ev->button() == Qt::RightButton)
                fMotionMode = MODE_PAN;
        break;
        case Qt::ControlModifier:
            fMotionMode = MODE_ZOOM;
    }
    
    switch(fMotionMode) {
        case MODE_ROLL:
            if(fTrackObject) {
                GLdouble x, y, z;
                gluProject(fCenter.x(), fCenter.y(), fCenter.z(),
                           fGLModelview, fGLProjection, fGLViewport,
                           &x, &y, &z);
                fCenterX = (int) ceil(x - 0.5);
                fCenterY = height() - (int) ceil(y - 0.5);
            } else {
                fCenterX = width() / 2;
                fCenterY = height() / 2;
            }
            
            fLastRoll = rollAngle(ev->x(), ev->y());
        break;
        case MODE_PAN:
        case MODE_ZOOM:
        case MODE_ROTATE:
            fLastPos = ev->pos();
        break;
        default:
        break;
    }
    
    ev->accept();
}

void GLWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons() && fMotionMode != MODE_NONE) {
        switch(fMotionMode) {
            case MODE_ROTATE:
                rotateMotion(ev);
            break;
            case MODE_ROLL:
                rollMotion(ev);
            break;
            case MODE_PAN:
                panMotion(ev);
            break;
            case MODE_ZOOM:
                zoomMotion(ev);
            break;
            default:
            break;
        }
        
        updateCam();
        emit camChanged();
    }
    
    ev->accept();
}

void GLWidget::start()
{
    if(!fPaused) return;
    fPaused = false;
    fTimer->start();
}

void GLWidget::pause()
{
    if(fPaused) return;
    fPaused = true;
    fTimer->stop();
    updateGL();
}

void GLWidget::_updateCam()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    Vector3 up = getUpDir();
    if(fTrackObject) {
        glTranslated(fX, fY, 0.);
        Vector3 campos = getCamPos();
        gluLookAt(campos.x(), campos.y(), campos.z(),
                  fCenter.x(), fCenter.y(), fCenter.z(),
                  up.x(), up.y(), up.z());
    } else {
        Vector3 lookat = fCamPos - getCamDir();
        gluLookAt(fCamPos.x(), fCamPos.y(), fCamPos.z(),
                  lookat.x(), lookat.y(), lookat.z(),
                  up.x(), up.y(), up.z());
    }
    
    glGetDoublev(GL_MODELVIEW_MATRIX, fGLModelview);
}

void GLWidget::updateCam()
{
    _updateCam();
    if(fPaused)
        updateGL();
}

void GLWidget::normalizeAngles()
{
    // Normalize theta
    if(fTheta < 0) fTheta = 0;
    if(fTheta > PI_UNITS) fTheta = PI_UNITS;
    
    // Normalize phi
    fPhi %= (2 * PI_UNITS);
    if(fPhi < 0) fPhi += (2 * PI_UNITS);
}

void GLWidget::initializeGL()
{
    // Needed for projection shadow
    glEnable(GL_BLEND);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    // Needed because glScale() affects the normal vector
    glEnable(GL_NORMALIZE);
    
    // Set ambient/diffuse color via glColor()
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    
    const float Light0Position[] = {1., 1., 1., 0.};
    glLightfv(GL_LIGHT0, GL_POSITION, Light0Position);
    const float Light0Ambient[] = {0., 0., 0., 1. };
    glLightfv(GL_LIGHT0, GL_AMBIENT, Light0Ambient);
    const float Light0Diffuse[] = {1., 1., 1., 1.};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, Light0Diffuse);
    const float GlobalAmbient[] = {.2, .2, .2, 1.};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, GlobalAmbient);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    const SimulationState* state = fSimulation->GetState(fT);
    
    if(state)
        fCenter = fSimulation->GetState(fT)->GetCenter() + fCenterOffset;
    
    _updateCam();
    
    if(state)
        state->Draw(fDrawMode);
    drawCenter();
    drawStatusText();
}

void GLWidget::resizeGL(int w, int h)
{
    if(h == 0) h = 1;
    
    float ratio = (float) w / h;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glViewport(0, 0, w, h);
    glGetIntegerv(GL_VIEWPORT, fGLViewport);
    
    gluPerspective(45, ratio, 1, 1000);
    glGetDoublev(GL_PROJECTION_MATRIX, fGLProjection);
    
    updateCam();
}

void GLWidget::drawCenter()
{
    glDisable(GL_LIGHTING);
    
    glBegin(GL_LINES);
    glColor3fv(X_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector3::eX);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector3::eX);

    glColor3fv(Y_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector3::eY);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector3::eY);

    glColor3fv(Z_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector3::eZ);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector3::eZ);
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void GLWidget::drawStatusText()
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    
    std::ostringstream s;
    double t = (double) fT*fSimulation->GetTimestep();
    s << "Time: " << std::setprecision(2) << std::fixed << t << "s";
    if(fPaused)
        s << "   [PAUSED]";
    
    glColor3f(1., 1., 1.);
    renderText(2, QFontMetrics(QFont()).ascent(), s.str().c_str());
    
    glPopAttrib();
}

