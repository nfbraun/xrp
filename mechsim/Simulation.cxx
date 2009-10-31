#include "Simulation.h"

const double Simulation::TIMESTEP = 0.01;

Simulation::Simulation()
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
    
    fTime = 0.0;
}

Simulation::~Simulation()
{
    dWorldDestroy(fWorld);
}

void Simulation::AdvanceTo(double t)
{
    while(fTime < t) {
        dWorldStep(fWorld, TIMESTEP);
        fTime += TIMESTEP;
    };
}

SimulationState Simulation::GetCurrentState()
{
    SimulationState state;
    const dReal* pos;
    
    state.fT = fTime;
    
    pos = dBodyGetPosition(fBall1);
    state.fX1 = pos[0];
    state.fY1 = pos[1];
    state.fZ1 = pos[2];
    
    pos = dBodyGetPosition(fBall2);
    state.fX2 = pos[0];
    state.fY2 = pos[1];
    state.fZ2 = pos[2];
    
    return state;
}
