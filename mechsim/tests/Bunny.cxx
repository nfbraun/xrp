#include "Bunny.h"
#include "GLHelper.h"
#include "STLReader.h"
#include <iostream>
#include <Eigen/Dense>

const int Bunny::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int Bunny::INT_PER_STEP = 16;
const char Bunny::TITLE[] = "Stanford Bunny";

void BunnyState::Draw(int) const
{
    glColor3f(1., 0., 1.);
    glPushMatrix();
    glTranslatef(2.*sin(fT/8.), 0., 0.);
    glScalef(10., 10., 10.);
    glCallList(fParent->GetBunnyList());
    glPopMatrix();
    
    GL::shadowsBeginFloor();
    GL::drawCheckerboardFloor();
    GL::shadowsBeginObjects(Eigen::Vector3d::UnitZ(), 0.);
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
    Eigen::Vector3f c(-0.095+0.156/2., 0.033, -0.062+0.121/2.);
    
    glRotatef(90., 1., 0., 0.);
    //glRotatef(180., 0., 1., 0.);
    
    glBegin(GL_TRIANGLES);
    for(int i=0; i < reader.getNTri(); ++i) {
        STLReader::STLTriRecord_t* tri = reader.getTriRecord(i);
        if(!tri) {
            std::cerr << __func__ << ": failed to read triangle " << i << std::endl;
            break;
        }
        Eigen::Vector3f v1(Eigen::Map<Eigen::Vector3f>(tri->v1));
        Eigen::Vector3f v2(Eigen::Map<Eigen::Vector3f>(tri->v2));
        Eigen::Vector3f v3(Eigen::Map<Eigen::Vector3f>(tri->v3));
        
        Eigen::Vector3f normal = -((v1-v2).cross(v1-v3)).normalized();
        GL::Normal3(normal);
        GL::Vertex3(v1-c);
        GL::Vertex3(v2-c);
        GL::Vertex3(v3-c);
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
