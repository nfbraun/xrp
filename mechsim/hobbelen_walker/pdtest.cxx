#include <iostream>
#include "Vector.h"
#include <ode/ode.h>

const int STEPS_PER_SEC = 1000;
const double PHI_0 = .1;
const double M = .5;
const double G = 9.81;
const double L = 5.;
const double Ixx = 1.5;
const double K0 = 50.;
double K1;

void TorqueControl(dBodyID body, dJointID joint)
{
    Vector3 pos(dBodyGetPosition(body)), avel(dBodyGetAngularVel(body));
    double ang = atan2(pos.x(), pos.z());
    double torque;
    torque = -K0 * ang - K1 * avel.y();
    dJointAddHingeTorque(joint, torque);
}

int main()
{
    dWorldID fWorld;
    dInitODE();
    
    fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -G);
    
    dMass mass;
    dMassSetParameters(&mass, M, 0., 0., 0., Ixx, Ixx, Ixx, 0., 0., 0.);
    
    dBodyID fBody = dBodyCreate(fWorld);
    dBodySetPosition(fBody, L * sin(PHI_0), 0., L * cos(PHI_0));
    dBodySetMass(fBody, &mass);
    
    dJointID fJoint = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJoint, fBody, 0);
    dJointSetHingeAnchor(fJoint, 0., 0., 0.);
    dJointSetHingeAxis(fJoint, 0., 1., 0.);
    
    K1 = 2.*sqrt((K0 - M*G*L)*(M*L*L + Ixx));
    
    for(int step=0; step<60*STEPS_PER_SEC; ++step) {
        if(step % 16 == 0) {
            Vector3 pos(dBodyGetPosition(fBody));
            
            printf("%.6f %.6f\n", (double)step/STEPS_PER_SEC, pos.x());
        }
        TorqueControl(fBody, fJoint);
        dWorldStep(fWorld, 1./STEPS_PER_SEC);
    }
    
    return 0;
}
