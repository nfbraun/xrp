#include "DoublePendulum.h"
#include "GLHelper.h"
#include <iostream>

const int DoublePendulum::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int DoublePendulum::INT_PER_STEP = 16;
const char DoublePendulum::TITLE[] = "Double pendulum";

void DPState::Draw() const
{
    GL::drawSphere(.2, fB1_pos);
    GL::drawSphere(.2, fB2_pos);
    
    GL::drawTube(.1, Vector3::Null, fB1_pos);
    GL::drawTube(.1, fB1_pos, fB2_pos);
}

DoublePendulum::DoublePendulum()
{
    fWorld = dWorldCreate();
    
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fBall1 = dBodyCreate(fWorld);
    dMass m;

    dMassSetSphere(&m, 1000.0, 0.1);
    dBodySetMass(fBall1, &m);
    dBodySetPosition(fBall1, 1., 0., 1.);
    
    fBall2 = dBodyCreate(fWorld);

    dMassSetSphere(&m, 1000.0, 0.1);
    dBodySetMass(fBall2, &m);
    dBodySetPosition(fBall2, 2., 0., 2.);

    dJointID j1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j1, fBall1, 0);
    dJointSetHingeAnchor(j1, 0.,0.,0.);
    dJointSetHingeAxis(j1, 0.,1.,0.);
    
    dJointID j2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(j2, fBall1, fBall2);
    dJointSetHingeAnchor(j2, 1., 0., 1.);
    dJointSetHingeAxis(j2, 0.,1.,0.);
    
    fCurStep = 0;
}

DoublePendulum::~DoublePendulum()
{
    dWorldDestroy(fWorld);
}

void DoublePendulum::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        ++fCurStep;
    }
}

DPState DoublePendulum::GetCurrentState()
{
    DPState state;
    const dReal* pos;
    
    state.fT = fCurStep / STEP_PER_SEC;
    
    pos = dBodyGetPosition(fBall1);
    state.fB1_pos = Vector3(pos);
    
    pos = dBodyGetPosition(fBall2);
    state.fB2_pos = Vector3(pos);
    
    return state;
}
