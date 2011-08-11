#include "BallSlide.h"
#include "GLHelper.h"
#include <GL/gl.h>

const int BallSlide::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int BallSlide::INT_PER_STEP = 16;
const char BallSlide::TITLE[] = "Ball drop";

dGeomID gSlideG;

void near_callback(void* data, dGeomID g1, dGeomID g2)
{
    const int MAX_CONTACTS = 4;
    dContact contact[MAX_CONTACTS];
    
    int num_contacts = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));

    for(int i=0; i<num_contacts; ++i) {
        contact[i].surface.mode = dContactBounce;
        contact[i].surface.bounce = 0.9;
        contact[i].surface.mu = 100.0;
        dJointID c = dJointCreateContact(((BallSlide*)data)->fWorld,
                                         ((BallSlide*)data)->fContactGroup,
                                         &contact[i]);
        dJointAttach(c, dGeomGetBody(g1), dGeomGetBody(g2));
    }
}

void BSState::Draw(int) const
{
    glColor3f(1., 1., 0.);
    GL::drawSphere(.3, fBPos);
    
    GL::shadowsBeginFloor();
    glColor3f(0., 1., 0.);
    GL::drawODEBox(gSlideG, 3., 1., .2);
    Vector3 n(sin(.2), 0., cos(.2));
    GL::shadowsBeginObjects(n, VectorOp::dot(n, Vector3(0., 0., 9.) + .1 * n));
    GL::drawSphere(.3, fBPos);
    GL::shadowsEnd();
    
    GL::shadowsBeginFloor();
    GL::drawCheckerboardFloor();
    GL::shadowsBeginObjects(Vector3(0., 0., 1.), 0.);
    GL::drawSphere(.3, fBPos);
    GL::shadowsEnd();
}

BallSlide::BallSlide()
{
    dInitODE();
    
    fWorld = dWorldCreate();
    fSpace = dSimpleSpaceCreate(0);
    
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fFloorG = dCreatePlane(fSpace, 0., 0., 1., 0.);
    fSlideG = dCreateBox(fSpace, 3., 1., .2);
    gSlideG = fSlideG;
    dGeomSetPosition(fSlideG, 0., 0., 9.);
    dQuaternion q;
    dQFromAxisAndAngle(q, 0., 1., 0., .2);
    dGeomSetQuaternion(fSlideG, q);
    
    fBall = dBodyCreate(fWorld);
    dMass m;

    dMassSetSphere(&m, 1000.0, 0.2);
    dBodySetMass(fBall, &m);
    dBodySetPosition(fBall, 0., 0., 10.);
    
    fBallG = dCreateSphere(fSpace, 0.2);
    dGeomSetBody(fBallG, fBall);
    
    fContactGroup = dJointGroupCreate(0);
}

BallSlide::~BallSlide()
{
    dWorldDestroy(fWorld);
    dSpaceDestroy(fSpace);
    dJointGroupDestroy(fContactGroup);
}

void BallSlide::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        dSpaceCollide(fSpace, this, near_callback);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        dJointGroupEmpty(fContactGroup);
    }
    // Waste time
    usleep(20000);
}

BSState BallSlide::GetCurrentState()
{
    BSState state;
    
    state.fBPos = Vector3(dBodyGetPosition(fBall));
    
    return state;
}
