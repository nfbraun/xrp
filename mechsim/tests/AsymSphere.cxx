#include "AsymSphere.h"
#include "GLHelper.h"
#include <GL/gl.h>

// ** Simulation parameters **
const int AsymSphere::STEP_PER_SEC = 16;
const int AsymSphere::INT_PER_STEP = 16;   // Integration intervals per timestep
const char AsymSphere::TITLE[] = "Moment of inertia test";

// ** System parameters **
const double AsymSphere::GAMMA   = 0.1;   // Floor slope
const double AsymSphere::R = .5;
const double AsymSphere::FLOOR_DIST = R;

// Width of slide
const double ASState::DISP_SLIDEWIDTH = 1.0;
// Half-length of slide
const int ASState::DISP_SLIDELEN2 = 3;

void ASState::Draw() const
{
    glMatrixMode(GL_MODELVIEW);
    
    DrawSlide();
    
    glPushMatrix();
    GL::Translate(fPos);
    GL::Rotate(fRot);
    GL::drawSphere(AsymSphere::R, Vector3::Null);
    glPopMatrix();
}

void ASState::DrawSlide() const
{
    float x1, z1, x2, z2;
    const float xn = sin(AsymSphere::GAMMA);
    const float zn = cos(AsymSphere::GAMMA);
    
    glNormal3f(xn, 0., zn);
    
    glBegin(GL_QUADS);
    for(int k=-DISP_SLIDELEN2; k<DISP_SLIDELEN2; k++) {
        x1 = -AsymSphere::FLOOR_DIST * xn + k * zn;
        z1 = -AsymSphere::FLOOR_DIST * zn - k * xn;
        x2 = x1 + zn;
        z2 = z1 - xn;
        
        if(k % 2)
            glColor3f(1., 0., 0.);
        else
            glColor3f(0., 0., 1.);
        glVertex3f(x1, DISP_SLIDEWIDTH/2., z1);
        glVertex3f(x1, 0, z1);
        glVertex3f(x2, 0, z2);
        glVertex3f(x2, DISP_SLIDEWIDTH/2., z2);
        
        if(k % 2)
            glColor3f(0., 0., 1.);
        else
            glColor3f(1., 0., 0.);
        glVertex3f(x1, 0, z1);
        glVertex3f(x1, -DISP_SLIDEWIDTH/2., z1);
        glVertex3f(x2, -DISP_SLIDEWIDTH/2., z2);
        glVertex3f(x2, 0, z2);
    }
    glEnd();
}

AsymSphere::AsymSphere()
{
    const double d = 0.4;
    const double phi0 = 0.;
    
    dMass mass;
    
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fFloorG = dCreatePlane(0, sin(GAMMA), 0., cos(GAMMA), -FLOOR_DIST);
    
    fBody = dBodyCreate(fWorld);
    dBodySetPosition(fBody, d*sin(phi0), 0., d*cos(phi0));
    dMassSetSphereTotal(&mass, 1., 1.);
    dBodySetMass(fBody, &mass);
    
    fBodyG = dCreateSphere(0, R);
    dGeomSetBody(fBodyG, fBody);
    dGeomSetOffsetPosition(fBodyG, -d*sin(phi0), 0., -d*cos(phi0));
    
    // dBodySetLinearVel(fBody, 0.1, 0.0, 0.0);
    
    fContactGroup = dJointGroupCreate(0);
}

AsymSphere::~AsymSphere()
{
    dWorldDestroy(fWorld);
    dJointGroupDestroy(fContactGroup);
}

void AsymSphere::Collide(dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(fWorld, fContactGroup, &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
}

void AsymSphere::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        Collide(fFloorG, fBodyG);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
}

ASState AsymSphere::GetCurrentState()
{
    ASState state;
    const Vector3 par(cos(AsymSphere::GAMMA), 0., -sin(AsymSphere::GAMMA));
    const Vector3 nor(sin(AsymSphere::GAMMA), 0.,  cos(AsymSphere::GAMMA));
    
    state.fRot = Rotation::FromQuatArray(dBodyGetQuaternion(fBody));
    state.fCoG = Vector3(dBodyGetPosition(fBody));
    state.fVel = Vector3(dBodyGetLinearVel(fBody));
    state.fOmega = Vector3(dBodyGetAngularVel(fBody));
    
    state.fPos = Vector3(dBodyGetPosition(fBody))
            + state.fRot * dGeomGetOffsetPosition(fBodyG);
    state.fPhi = VectorOp::dot(state.fPos, par) / AsymSphere::R;
    state.fHeight = VectorOp::dot(state.fPos, nor);
    
    return state;
}
