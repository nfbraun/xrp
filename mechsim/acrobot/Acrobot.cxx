#include "Acrobot.h"
#include "GLHelper.h"
#include "ODEHelper.h"
#include <cmath>
#include <iostream>

const int Acrobot::STEP_PER_SEC = 16;
// Integration intervals per timestep
const int Acrobot::INT_PER_STEP = 160;
const char Acrobot::TITLE[] = "Acrobot";

void AcroState::Draw(int) const
{
    GL::drawSphere(.1, fB1_pos);
    GL::drawSphere(.1, fB2_pos);
    
    GL::drawTube(.05, Eigen::Vector3d::Zero(), fB1_pos);
    GL::drawTube(.05, fB1_pos, fB2_pos);
}

Acrobot::Acrobot()
{
    const double THETA_1_INI = M_PI/2., THETA_2_INI = 0.0;
    
    const double G = 9.81;
    const double M1 = 1.;
    const double M2 = 1.;
    const double LC = 1.;
    const double L1 = 1.;
    const double L2 = 1.;
    const double I1 = 1.;
    const double I2 = 1.;
    
    fWorld = dWorldCreate();
    
    dWorldSetGravity(fWorld, 0., 0., -9.81);
    
    fPart1 = dBodyCreate(fWorld);
    dMass m;

    dMassSetParameters(&m, M1,
                       0.0, 0.0, 0.0,
                       1.0, I1, 1.0,
                       0.0, 0.0, 0.0);
    dBodySetMass(fPart1, &m);
    dBodySetPosition(fPart1, LC*sin(THETA_1_INI), 0., LC*cos(THETA_1_INI));
    
    fPart2 = dBodyCreate(fWorld);

    dMassSetParameters(&m, M2,
                       0.0, 0.0, 0.0,
                       1.0, I2, 1.0,
                       0.0, 0.0, 0.0);
    dBodySetMass(fPart2, &m);
    dBodySetPosition(fPart2, L1*sin(THETA_1_INI) + L2*sin(THETA_1_INI + THETA_2_INI),
                             0.,
                             L1*cos(THETA_1_INI) + L2*cos(THETA_1_INI + THETA_2_INI));
    //dBodySetLinearVel(fBall2, 0., 0., sqrt(4.*9.81));

    fJ1 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJ1, fPart1, 0);
    dJointSetHingeAnchor(fJ1, 0.,0.,0.);
    dJointSetHingeAxis(fJ1, 0.,1.,0.);
    
    fJ2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJ2, fPart1, fPart2);
    dJointSetHingeAnchor(fJ2, L1*sin(THETA_1_INI), 0., L2*cos(THETA_1_INI));
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
    
    state.fT = fCurStep / STEP_PER_SEC;
    
    state.fB1_pos = ODE::BodyGetPosition(fPart1);
    state.fB2_pos = ODE::BodyGetPosition(fPart2);
    
    return state;
}
