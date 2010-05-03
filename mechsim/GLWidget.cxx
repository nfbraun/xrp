#include "GLWidget.h"
#include <QtOpenGL>
#include <QString>
#include <QFont>
#include <QFontMetrics>
#include <GL/glu.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>

const double GLWidget::RAD_TO_DEG = 180. / M_PI;
const double GLWidget::DEG_TO_RAD = M_PI / 180.;
const int GLWidget::ANG_STEPS_PER_PI = 3 * 180;
const double GLWidget::ANG_STEP = M_PI / ANG_STEPS_PER_PI;
const double GLWidget::WHEEL_STEP = 0.01;

const Vector3 GLWidget::DEFAULT_CAM_POS = Vector3(0, 10., 0.);

GLWidget::GLWidget(Simulation* sim, QWidget* parent)
    : QGLWidget(parent), fSimulation(sim)
{
    fTimer = new QTimer(this);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(timestep()));
    fTimer->start(sim->GetTimestep()*1000.);
    
    fT = 0;
    
    fCamPos = DEFAULT_CAM_POS;
    
    Vector3 look = Vector::Null - fCamPos;
    fPhi = look.phi() / ANG_STEP;
    fTheta = look.theta() / ANG_STEP;
    normalizeAngles();
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

void GLWidget::setCamPos(double x, double y, double z)
{
    fCamPos = Vector3(x, y, z);
    updateCam();
}

void GLWidget::setCamPhi(double phi)
{
    fPhi = (int) ceil((phi * DEG_TO_RAD) / ANG_STEP - .5);
    normalizeAngles();
    updateCam();
}

void GLWidget::setCamTheta(double theta)
{
    fTheta = (int) ceil((theta * DEG_TO_RAD) / ANG_STEP - .5);
    normalizeAngles();
    updateCam();
}

void GLWidget::resetCamPos()
{
    fCamPos = DEFAULT_CAM_POS;
    Vector3 look = Vector::Null - fCamPos;
    fPhi = look.phi() / ANG_STEP;
    fTheta = look.theta() / ANG_STEP;
    
    updateCam();
}

void GLWidget::normalizeAngles()
{
    // Normalize theta
    if(fTheta < 0) fTheta = 0;
    if(fTheta > ANG_STEPS_PER_PI) fTheta = ANG_STEPS_PER_PI;
    
    // Normalize phi
    fPhi %= (2 * ANG_STEPS_PER_PI);
    if(fPhi < 0) fPhi += (2 * ANG_STEPS_PER_PI);
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
    Vector3 d = Vector3::Spherical(1., fTheta*ANG_STEP, fPhi*ANG_STEP);
    d *= (ev->delta() * WHEEL_STEP);
    fCamPos += d;
    
    updateCam();
    emit camChanged();
    
    ev->accept();
}

void GLWidget::mousePressEvent(QMouseEvent* ev)
{
    fLastPos = ev->pos();
    ev->accept();
}

void GLWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons() & Qt::LeftButton) {
        int dx = ev->x() - fLastPos.x();
        int dy = ev->y() - fLastPos.y();
        
        fPhi += dx;
        fTheta -= dy;
        
        normalizeAngles();
        updateCam();
        emit camChanged();
        
    }
    fLastPos = ev->pos();

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
    Vector3 lookAt = fCamPos;
    lookAt += Vector3::Spherical(1., fTheta*ANG_STEP, fPhi*ANG_STEP);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(fCamPos.x(), fCamPos.y(), fCamPos.z(),
              lookAt.x(), lookAt.y(), lookAt.z(),
              0., 0., 1.);
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

    const float Light0Position[] = {1., 1., 0., 0.};
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
    
    fSimulation->GetState(fT)->Draw();
    drawStatusText();
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
    
    gluPerspective(45, ratio, 1, 1000);
    
    updateCam();
}
