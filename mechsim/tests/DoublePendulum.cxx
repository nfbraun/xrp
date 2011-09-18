#include "DoublePendulum.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include <iostream>

const int DoublePendulum::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int DoublePendulum::INT_PER_STEP = 16;
const char DoublePendulum::TITLE[] = "Double pendulum";

void DPState::Draw(int) const
{
    GL::drawSphere(.2, fB1_pos);
    GL::drawSphere(.2, fB2_pos);
    
    GL::drawTube(.1, Eigen::Vector3d::Zero(), fB1_pos);
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

    fJoint1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJoint1, fBall1, 0);
    dJointSetHingeAnchor(fJoint1, 0.,0.,0.);
    dJointSetHingeAxis(fJoint1, 0.,1.,0.);
    
    fJoint2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJoint2, fBall1, fBall2);
    dJointSetHingeAnchor(fJoint2, 1., 0., 1.);
    dJointSetHingeAxis(fJoint2, 0.,1.,0.);
    
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
    
    state.fT = fCurStep / STEP_PER_SEC;
    
    state.fB1_pos = ODE::BodyGetPosition(fBall1);
    state.fB2_pos = ODE::BodyGetPosition(fBall2);
    
    state.fOmega1 = dJointGetHingeAngleRate(fJoint1);
    state.fOmega2 = dJointGetHingeAngleRate(fJoint2);
    
    return state;
}
