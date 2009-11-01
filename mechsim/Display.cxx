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



void DisplayCallback()




void ReshapeCallback(int w, int h)


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
