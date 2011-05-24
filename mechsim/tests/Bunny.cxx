#include "Bunny.h"
#include "GLHelper.h"
#include "STLReader.h"
#include "Vector.h"
#include <iostream>

const int Bunny::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int Bunny::INT_PER_STEP = 16;
const char Bunny::TITLE[] = "Stanford Bunny";

void BunnyState::Draw() const
{
    glColor3f(1., 0., 1.);
    glPushMatrix();
    glTranslatef(2.*sin(fT/8.), 0., 0.);
    glScalef(10., 10., 10.);
    glCallList(fParent->GetBunnyList());
    glPopMatrix();
    
    GL::shadowsBeginFloor();
    GL::drawCheckerboardFloor();
    GL::shadowsBeginObjects(Vector3(0., 0., 1.), 0.);
    glPushMatrix();
    glTranslatef(2.*sin(fT/8.), 0., 0.);
    glScalef(10., 10., 10.);
    glCallList(fParent->GetBunnyList());
    glPopMatrix();
    GL::shadowsEnd();
}

Bunny::Bunny()
    : fCurStep(0), fGLList(0)
{
}

GLuint Bunny::GetBunnyList()
{
    if(fGLList > 0) return fGLList;
    
    STLReader reader("bunny.stl");
    if(!reader) {
        std::cerr << __func__ << ": read STL file failed" << std::endl;
        return 0;
    }
    
    fGLList = glGenLists(1);
    if(fGLList == 0) {
        std::cerr << __func__ << ": glGenLists() failed" << std::endl;
        return 0;
    }
    
    glNewList(fGLList, GL_COMPILE);
    Vector3f c(-0.095+0.156/2., 0.033, -0.062+0.121/2.);
    
    glRotatef(90., 1., 0., 0.);
    //glRotatef(180., 0., 1., 0.);
    
    glBegin(GL_TRIANGLES);
    for(int i=0; i < reader.getNTri(); ++i) {
        STLReader::STLTriRecord_t* tri = reader.getTriRecord(i);
        if(!tri) {
            std::cerr << __func__ << ": failed to read triangle " << i << std::endl;
            break;
        }
        Vector3f v1(tri->v1), v2(tri->v2), v3(tri->v3);
        Vector3f normal = -VectorOp::cross(v1-v2, v1-v3).norm();
        glNormal3fv(normal);
        glVertex3fv(v1-c);
        glVertex3fv(v2-c);
        glVertex3fv(v3-c);
    }
    glEnd();
    
    glEndList();
    
    return fGLList;
}

BunnyState Bunny::GetCurrentState()
{
    BunnyState state;
    state.fParent = this;
    state.fT = fCurStep;
    return state;
}
