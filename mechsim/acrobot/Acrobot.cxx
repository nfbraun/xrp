#include "Acrobot.h"
#include "GLHelper.h"
#include <cmath>
#include <iostream>

const int Acrobot::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int Acrobot::INT_PER_STEP = 160;
const char Acrobot::TITLE[] = "Acrobot";

void AcroState::Draw() const
{
    GL::drawSphere(.1, fB1_pos);
    GL::drawSphere(.1, fB2_pos);
    
    GL::drawTube(.05, Vector3::Null, fB1_pos);
    GL::drawTube(.05, fB1_pos, fB2_pos);
}

Acrobot::Acrobot()
{
    const double THETA_1_INI = M_PI/2., THETA_2_INI = 0.0;
    
    fWorld = dWorldCreate();
    
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fBall1 = dBodyCreate(fWorld);
    dMass m;

    dMassSetSphereTotal(&m, 1.0, 0.00001);
    dBodySetMass(fBall1, &m);
    dBodySetPosition(fBall1, sin(THETA_1_INI), 0., cos(THETA_1_INI));
    
    fBall2 = dBodyCreate(fWorld);

    dMassSetSphereTotal(&m, 1.0, 0.00001);
    dBodySetMass(fBall2, &m);
    dBodySetPosition(fBall2, sin(THETA_1_INI) + sin(THETA_1_INI + THETA_2_INI),
                             0.,
                             cos(THETA_1_INI) + cos(THETA_1_INI + THETA_2_INI));
    dBodySetLinearVel(fBall2, 0., 0., sqrt(4.*9.81));

    fJ1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJ1, fBall1, 0);
    dJointSetHingeAnchor(fJ1, 0.,0.,0.);
    dJointSetHingeAxis(fJ1, 0.,1.,0.);
    
    fJ2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJ2, fBall1, fBall2);
    dJointSetHingeAnchor(fJ2, sin(THETA_1_INI), 0., cos(THETA_1_INI));
    dJointSetHingeAxis(fJ2, 0.,1.,0.);
    
    fCurStep = 0;
}

Acrobot::~Acrobot()
{
    dWorldDestroy(fWorld);
}

void Acrobot::HingeFriction(dJointID j)
{
    const double GAMMA = 0.0; //0.1;
    dJointAddHingeTorque(j, -GAMMA * dJointGetHingeAngleRate(j));
}

void Acrobot::Advance()
{
    for(int i=0; i<INT_PER_STEP; ++i) {
        HingeFriction(fJ1);
        HingeFriction(fJ2);
        dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        ++fCurStep;
    }
}

AcroState Acrobot::GetCurrentState()
{
    AcroState state;
    const dReal* pos;
    
    state.fT = fCurStep / STEP_PER_SEC;
    
    pos = dBodyGetPosition(fBall1);
    state.fB1_pos = Vector3(pos);
    
    pos = dBodyGetPosition(fBall2);
    state.fB2_pos = Vector3(pos);
    
    return state;
}
