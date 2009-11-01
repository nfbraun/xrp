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

const double GLWidget::RAD_TO_DEG = 180. / M_PI;
const double GLWidget::DEG_TO_RAD = M_PI / 180.;
const double GLWidget::ANG_STEP = M_PI / (180. * 3.);

GLWidget::GLWidget(Simulation* sim, QWidget* parent)
    : QGLWidget(parent), fSimulation(sim)
{
    fTimer = new QTimer(this);
    connect(fTimer, SIGNAL(timeout()), this, SLOT(timestep()));
    fTimer->start(sim->GetTimestep()*1000.);
    
    fT = 0;
    
    fCamPos = Vector3(0., 10., 0.);
    
    Vector3 look = Vector::Null - fCamPos;
    fPhi = look.phi() / ANG_STEP;
    fTheta = look.theta() / ANG_STEP;
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

void GLWidget::timestep()
{
    ++fT;
    updateGL();
    emit timeChanged(fT);
}

void GLWidget::setTime(int t)
{
    fT = t;
    updateGL();
}

void GLWidget::mousePressEvent(QMouseEvent* ev)
{
    fLastPos = ev->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent* ev)
{
    if(ev->buttons() & Qt::LeftButton) {
        int dx = ev->x() - fLastPos.x();
        int dy = ev->y() - fLastPos.y();
        
        fPhi += dx;
        fTheta -= dy;
        
        camPosChanged();
    }
    fLastPos = ev->pos();
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

void GLWidget::camPosChanged()
{
    Vector3 lookAt = fCamPos;
    lookAt += Vector3::Spherical(1., fTheta*ANG_STEP, fPhi*ANG_STEP);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(fCamPos.x(), fCamPos.y(), fCamPos.z(),
              lookAt.x(), lookAt.y(), lookAt.z(),
              0., 0., 1.);
    updateGL();
}

void GLWidget::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
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
    
    renderText(2, QFontMetrics(QFont()).ascent(), s.str().c_str());
    
    glPopAttrib();
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

void GLWidget::resizeGL(int w, int h)
{
    if(h == 0) h = 1;
    
    float ratio = (float) w / h;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glViewport(0, 0, w, h);
    
    gluPerspective(45, ratio, 1, 1000);
    
    camPosChanged();
}
