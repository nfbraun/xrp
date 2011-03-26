#include "Cube.h"
#include "GLHelper.h"
#include "Vector.h"
#include <iostream>

const int Cube::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int Cube::INT_PER_STEP = 16;
const char Cube::TITLE[] = "Shadow test";

void CubeState::Draw() const
{
    glColor3f(.7, .7, .7);
    glPushMatrix();
    glTranslatef(0.,0.,1.);
    GL::drawUnitCube();
    // GL::drawSphere(0.5, Vector3::Null);
    glPopMatrix();
    
    GL::shadowsBeginFloor();
    glPushMatrix();
    glTranslatef(0., 0., -10.);
    GL::drawCheckerboardFloor();
    glPopMatrix();
    GL::shadowsBeginObjects(Vector3(0., 0., 1.), -10.);
    glPushMatrix();
    glTranslatef(0.,0.,1.);
    GL::drawUnitCube();
    glPopMatrix();
    GL::shadowsEnd();
}

Cube::Cube()
    : fCurStep(0)
{
}

CubeState Cube::GetCurrentState()
{
    CubeState state;
    state.fParent = this;
    state.fT = fCurStep;
    return state;
}
