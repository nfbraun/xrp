#include "Display.h"
#include "Vector.h"

#include <sys/time.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace Display {

Simulation simulation;

const double TIMESTEP = 0.05;
double nextDisplay;
double startTime;

const double RAD_TO_DEG = 180. / M_PI;

void SolidCylinder(double r, Vector3 p1, Vector3 p2)
{
    Vector3 d = p2 - p1;
    double angle = asin(sqrt(d.x*d.x + d.y*d.y) / d.mag()) * RAD_TO_DEG;
    if(d.z < 0) angle = 180. - angle;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(p1.x, p1.y, p1.z);
    glRotatef(angle, -d.y, d.x, 0.);
    glutSolidCylinder(r, d.mag(), 8, 4);
    glPopMatrix();
}

void DrawText(const std::string& text)
{
    for(std::string::const_iterator ch = text.begin(); ch != text.end(); ++ch)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *ch);
}

void DisplayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    SimulationState state = simulation.GetCurrentState();
    
    glPushMatrix();
    glTranslatef(state.fX1, state.fY1, state.fZ1);
    glutSolidSphere(0.2, 16, 16);
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(state.fX2, state.fY2, state.fZ2);
    glutSolidSphere(0.2, 16, 16);
    glPopMatrix();
    
    SolidCylinder(.1,
                  Vector3(0., 0., 0.),
                  Vector3(state.fX1, state.fY1, state.fZ1));
    
    SolidCylinder(.1,
                  Vector3(state.fX1, state.fY1, state.fZ1),
                  Vector3(state.fX2, state.fY2, state.fZ2));
    
    DrawStatusText(state);
    
    glutSwapBuffers();
}

void DrawStatusText(const SimulationState& state)
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-.5, 639.5, -.5, 479.5, -1., 1.);
    
    glRasterPos2i(0, 470);
    std::ostringstream s;
    s << "Time: " << std::setprecision(2) << std::fixed << state.fT << "s";
    DrawText(s.str());
    
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    
    glPopAttrib();
}

void ReshapeCallback(int w, int h)
{
    if(h == 0) h = 1;
    
    float ratio = (float) w / h;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glViewport(0, 0, w, h);
    
    gluPerspective(45, ratio, 1, 1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0., 10., 0.,
              0., 0., 0.,
              0., 0., 1.);
}

void IdleCallback()
{
    simulation.AdvanceTo(nextDisplay - startTime);
    
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double curTime = tv.tv_sec + (double) tv.tv_usec / 1e6;
    if(curTime < nextDisplay)
        usleep((nextDisplay - curTime)*1000000);
    else
        std::cerr << "Missed display deadline by " << (curTime - nextDisplay) << " s." << std::endl;
        
    glutPostRedisplay();
    
    nextDisplay += TIMESTEP;
}

void InitGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

void Init()
{
    InitGL();
    glutDisplayFunc(DisplayCallback);
    glutReshapeFunc(ReshapeCallback);
    glutIdleFunc(IdleCallback);
    
    // Schedule first display
    struct timeval tv;
    gettimeofday(&tv, NULL);
    startTime = (tv.tv_sec + (double) tv.tv_usec / 1e6);
    nextDisplay = Display::startTime + Display::TIMESTEP;
}

void Run()
{
    glutMainLoop();
}

} // end namespace Display
