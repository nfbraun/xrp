#include "Simulation.h"
#include "Display.h"
#include <GL/glut.h>

Simulation simulation;

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowPosition(1280, 0);
    glutInitWindowSize(640, 480);
    glutCreateWindow("Double pendulum test");
    
    Display::Init();
    Display::Run();
    
    return 0;
}
