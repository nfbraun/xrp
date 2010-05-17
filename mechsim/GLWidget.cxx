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
const int GLWidget::ANG_STEPS_PER_PI = 90;
const double GLWidget::ANG_STEP = M_PI / ANG_STEPS_PER_PI;
const double GLWidget::DIST_STEP = 0.01;
const int GLWidget::DIST_WHEEL_CORRECTION = 10;
const double GLWidget::SHIFT_STEP = 0.1;

const double GLWidget::AXIS_LENGTH = 1.0;
const float GLWidget::X_AXIS_COLOR[] = { 1.0, 0.0, 0.0 };
const float GLWidget::Y_AXIS_COLOR[] = { 1.0, 1.0, 0.0 };
const float GLWidget::Z_AXIS_COLOR[] = { 0.0, 1.0, 0.0 };

const int GLWidget::DEFAULT_CAM_DIST = 5. / DIST_STEP;

GLWidget::GLWidget(Simulation* sim, QWidget* parent)
    : QGLWidget(parent),
      fX(0.), fZ(0.),
      fDist(DEFAULT_CAM_DIST),
      fRotation(Rotation::Unit),
      fT(0),
      fSimulation(sim)
{
    fTimer = new QTimer(this);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(timestep()));
    fTimer->start(sim->GetTimestep()*1000.);
    
    // fCenter = fSimulation->GetCenter();
    
    fPaused = false;
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

void GLWidget::setCamX(double x)
{
    fX = x;
    updateCam();
}

void GLWidget::setCamY(double y)
{
    fZ = y;
    updateCam();
}

void GLWidget::setCamDist(double d)
{
    double dist = log(d) / DIST_STEP;
    fDist = (int) ceil(dist - .5);
    updateCam();
}

void GLWidget::setCamRotation(const Rotation& r)
{
    fRotation = r;
    updateCam();
}

void GLWidget::rotateCam(const Rotation &r)
{
    fRotation *= r;
    updateCam();
}

void GLWidget::resetCamPos()
{
    fDist = DEFAULT_CAM_DIST;
    fRotation = Rotation::Unit;
    
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
    fT = t;
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent* ev)
{
    fDist -= ev->delta() / DIST_WHEEL_CORRECTION;
    
    updateCam();
    emit camChanged();
    
    ev->accept();
}

Vector3 GLWidget::trackballVector(int px, int py)
{
    // see http://www.opengl.org/wiki/Trackball
    const double r = 1.0;
    int d = std::min(width(), height());
    
    double x = (double) (2*px - width()) / d;
    double z = (double) (2*py - height()) / d;
    double y;
    
    if((x*x + z*z) < ((r*r)/2.)) {
        y = sqrt(r*r - x*x - z*z);
    } else {
        y = (r*r/2.) / sqrt(x*x + z*z);
    }
    
    return Vector3(x, y, z).norm();
}

void GLWidget::trackballMotion(QMouseEvent* ev)
{
    // see http://www.opengl.org/wiki/Trackball
    Vector3 v2 = trackballVector(ev->x(), ev->y());
    double dot = Vector::dot(fLastVec, v2);
    
    // alpha = acos(dot);
    // fRotation = Rotation(cos(alpha/2), naxis.norm() * sin(alpha/2));
    // Note that naxis.norm() = naxis / sin(alpha).
    
    Vector3 naxis = Vector::cross(fLastVec, v2) * sqrt(1./(2.+2.*dot));
    
    fRotation *= Rotation(sqrt((1.+dot)/2.), naxis.x(), naxis.y(), naxis.z());
    
    fLastVec = v2;
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
    
    fRotation *= Rotation(roll2 - fLastRoll, Vector3(0., 1., 0.));
    
    fLastRoll = roll2;
}

void GLWidget::panMotion(QMouseEvent* ev)
{
    GLdouble wx, wy, wz;
    gluProject(0., 0., 0., fGLModelview, fGLProjection, fGLViewport, &wx, &wy, &wz);
    
    GLdouble x1, y1, z1, x2, y2, z2;
    gluUnProject(fLastPos.x(), fLastPos.y(), wz,
                 fGLModelview, fGLProjection, fGLViewport,
                 &x1, &y1, &z1);
    gluUnProject(ev->x(), ev->y(), wz,
                 fGLModelview, fGLProjection, fGLViewport,
                 &x2, &y2, &z2);
    
    fX += x1 - x2;
    fZ += z2 - z1;
    
    fLastPos = ev->pos();
}

void GLWidget::zoomMotion(QMouseEvent* ev)
{
    fDist -= (ev->y() - fLastPos.y());
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
            if(ev->button() == Qt::LeftButton)
                fMotionMode = MODE_ROLL;
            else if(ev->button() == Qt::RightButton)
                fMotionMode = MODE_PAN;
        break;
        case Qt::ControlModifier:
            fMotionMode = MODE_ZOOM;
    }
    
    switch(fMotionMode) {
        case MODE_ROTATE:
            fLastVec = trackballVector(ev->x(), ev->y());
        break;
        case MODE_ROLL:
            fLastRoll = rollAngle(ev->x(), ev->y());
        break;
        case MODE_PAN:
        case MODE_ZOOM:
            fLastPos = ev->pos();
        break;
        default:
        break;
    }
    
    ev->accept();
}

void GLWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons()) {
        switch(fMotionMode) {
            case MODE_ROTATE:
                trackballMotion(ev);
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
    gluLookAt(fX, getCamDist(), fZ,
              fX, 0., fZ,
              0., 0., 1.);
    glGetDoublev(GL_MODELVIEW_MATRIX, fGLModelview);
    glTranslated(fCenter.x(), fCenter.y(), fCenter.z());
    RotateGL(fRotation);
    glTranslated(-fCenter.x(), -fCenter.y(), -fCenter.z());
    
    GLdouble x, y, z;
    gluProject(fCenter.x(), fCenter.y(), fCenter.z(), fGLModelview, fGLProjection, fGLViewport, &x, &y, &z);
    fCenterX = (int) ceil(x - 0.5);
    fCenterY = height() - (int) ceil(y - 0.5);
}

void GLWidget::updateCam()
{
    _updateCam();
    if(fPaused)
        updateGL();
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    const float Light0Position[] = {0., -5., 0., 0.};
    glLightfv(GL_LIGHT0, GL_POSITION, Light0Position);
    const float Light0Ambient[] = {0., 0., 0., 1. };
    glLightfv(GL_LIGHT0, GL_AMBIENT, Light0Ambient);
    const float Light0Diffuse[] = {10., 10., 10., 1.};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, Light0Diffuse);
    const float GlobalAmbient[] = {.2, .2, .2, 1.};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, GlobalAmbient);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glPushMatrix();
    
    fCenter = fSimulation->GetState(fT)->GetCenter();
    glTranslatef(-fCenter.x(), -fCenter.y(), -fCenter.z());
    fCenter += fCenterOffset;
    fSimulation->GetState(fT)->Draw();
    
    drawCenter();
    
    glPopMatrix();
    
    drawStatusText();
}

void GLWidget::drawCenter()
{
    glDisable(GL_LIGHTING);
    
    glBegin(GL_LINES);
    glColor3fv(X_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector::eX);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector::eX);

    glColor3fv(Y_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector::eY);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector::eY);

    glColor3fv(Z_AXIS_COLOR);
    glVertex3dv(fCenter - AXIS_LENGTH * Vector::eZ);
    glVertex3dv(fCenter + AXIS_LENGTH * Vector::eZ);
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

void GLWidget::drawDiscSegment(const double r, double h, const double alpha)
{
    const int n = 8;
    const double alpha2n = alpha / (2 * n);
    const double rc = r * cos(alpha / 2);
    double phi;
    double x1, x2, x3, x4;
    double y1, y2, y3, y4;
    
    h /= 2.;
    
    phi = -n * alpha2n;
    x3 = rc * tan(phi);
    y3 = rc;
    x4 = r * sin(phi);
    y4 = r * cos(phi);

    glBegin(GL_QUADS);
    for(int i=-n+1; i<=n; ++i) {
        phi = i * alpha2n;
    
        x1 = x3; y1 = y3; x2 = x4; y2 = y4;
        x3 = rc * tan(phi);
        y3 = rc;
        x4 = r * sin(phi);
        y4 = r * cos(phi);
        
        glNormal3f(0., 0., 1.);
        glVertex3f(x1, y1, h);
        glVertex3f(x2, y2, h);
        glVertex3f(x4, y4, h);
        glVertex3f(x3, y3, h);
        
        glNormal3f(0., 0., -1.);
        glVertex3f(x1, y1, -h);
        glVertex3f(x2, y2, -h);
        glVertex3f(x4, y4, -h);
        glVertex3f(x3, y3, -h);
        
        glNormal3f(-cos(phi), -sin(phi), 0.);
        glVertex3f(x1, y1, h);
        glVertex3f(x3, y3, h);
        glVertex3f(x3, y3, -h);
        glVertex3f(x1, y1, -h);
        
        glNormal3f(cos(phi), sin(phi), 0.);
        glVertex3f(x2, y2, h);
        glVertex3f(x4, y4, h);
        glVertex3f(x4, y4, -h);
        glVertex3f(x2, y2, -h);
    }
    glEnd();
}

void GLWidget::drawCheckerboardFloor()
{
    glBegin(GL_QUADS);
        glNormal3f(0., 0., 1.);
        
        for(int x=-5; x <= 5; ++x) {
            for(int y=-5; y <= 5; ++y) {
                if((x+y)%2)
                    glColor3f(1., 0., 0.);
                else
                    glColor3f(0., 0., 1.);
                
                glVertex3f((x-1)*10., (y-1)*10., 0.);
                glVertex3f( x*10.,    (y-1)*10., 0.);
                glVertex3f( x*10.,     y*10.,    0.);
                glVertex3f((x-1)*10.,  y*10.,    0.);
            }
        }
    glEnd();
}

void GLWidget::drawTube(double r, Vector3 p1, Vector3 p2)
{
    Vector3 d = p2 - p1;
    double angle = asin(sqrt(d.x()*d.x() + d.y()*d.y()) / d.mag()) * RAD_TO_DEG;
    if(d.z() < 0) angle = 180. - angle;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(p1.x(), p1.y(), p1.z());
    glRotatef(angle, -d.y(), d.x(), 0.);

    GLUquadric *quad = gluNewQuadric();
    gluCylinder(quad, r, r, d.mag(), 8, 4);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void GLWidget::drawSphere(double r, Vector3 p)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    glTranslatef(p.x(), p.y(), p.z());
    GLUquadric *quad = gluNewQuadric();
    gluSphere(quad, r, 16, 16);
    gluDeleteQuadric(quad);
    
    glPopMatrix();
}

void GLWidget::drawODEBox(dGeomID id, double lx, double ly, double lz)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    const dReal* pos = dGeomGetPosition(id);
    const dReal* rot = dGeomGetRotation(id);
    
    GLfloat mat[] = { rot[0], rot[4], rot[8], 0.,
                      rot[1], rot[5], rot[9], 0.,
                      rot[2], rot[6], rot[10], 0.,
                      pos[0], pos[1], pos[2], 1. };

    glMultMatrixf(mat);
    glScalef(lx, ly, lz);
    
    drawUnitCube();
    
    glPopMatrix();
}

void GLWidget::drawBox(Vector3 p1, Vector3 p2)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    
    Vector3 p = .5 * (p1 + p2);
    glTranslatef(p.x(), p.y(), p.z());
    Vector3 d = p2 - p1;
    glScalef(d.x(), d.y(), d.z());
    
    drawUnitCube();
    
    glPopMatrix();
}

void GLWidget::drawUnitCube()
{
    glBegin(GL_QUADS);
    
    glNormal3f(-1., 0., 0.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f(-.5, -.5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5,  .5, -.5);
    
    glNormal3f(0., 0., -1.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5,  .5, -.5);
    glVertex3f(-.5,  .5, -.5);
    
    glNormal3f(0., -1., 0.);
    glVertex3f(-.5, -.5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);
    glVertex3f(-.5, -.5,  .5);
    
    glNormal3f(1., 0., 0.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f( .5,  .5, -.5);
    glVertex3f( .5, -.5, -.5);
    glVertex3f( .5, -.5,  .5);

    glNormal3f(0., 0., 1.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5, -.5,  .5);
    glVertex3f( .5, -.5,  .5);

    glNormal3f(0., 1., 0.);
    glVertex3f( .5,  .5,  .5);
    glVertex3f(-.5,  .5,  .5);
    glVertex3f(-.5,  .5, -.5);
    glVertex3f( .5,  .5, -.5);

    glEnd();
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
