#include <iostream>
#include <ode/ode.h>

int main()
{
    const int STEP_PER_SEC = 25;
    const int INT_PER_STEP = 100;
    
    dMass mass;
    
    dInitODE();
    
    dWorldID fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -1.);
    
    dBodyID fBody1 = dBodyCreate(fWorld);
    dBodySetPosition(fBody1, 0., 0., -1.);
    dMassSetParameters(&mass, 1.,            // total mass
                              0., 0., 0.,    // center of gravity
                              1., .5, .7,    // I_11, I_22, I_33
                              0., 0., 0.);   // I_12, I_13, I_23
    dBodySetMass(fBody1, &mass);
    
    dBodyID fBody2 = dBodyCreate(fWorld);
    dBodySetPosition(fBody2, 0., 0., -2.);
    dMassSetParameters(&mass, .5,            // total mass
                              0., 0., 0.,    // center of gravity
                              .1, .2, .3,    // I_11, I_22, I_33
                              0., 0., 0.);   // I_12, I_13, I_23
    dBodySetMass(fBody2, &mass);
    
    dJointID fJoint1 = dJointCreateBall(fWorld, 0);
    dJointAttach(fJoint1, fBody1, 0);
    dJointSetBallAnchor(fJoint1, 0., 0., 0.);
    
    dJointID fJoint2 = dJointCreateHinge(fWorld, 0);
    dJointAttach(fJoint2, fBody2, fBody1);
    dJointSetHingeAnchor(fJoint2, 0., 0., -1.5);
    
    const double phi_0 = .7;
    const double theta_0 = -.5;
    
    /* dQuaternion q0 = { 1., 0., 0., 0. };
    dBodySetQuaternion(fBody, q0); */
    dBodySetLinearVel(fBody1, -theta_0, phi_0, 0.);
    dBodySetAngularVel(fBody1, phi_0, theta_0, 0.);
    dBodySetLinearVel(fBody2, -2.*theta_0, 2.*phi_0, 0.);
    dBodySetAngularVel(fBody2, phi_0, theta_0, 0.);
    
    //std::cout << "#:1:x\n#:2:y\n#:3:z\n";
    //std::cout << "#:4:x_p\n#:5:y_p\n#:6:z_p" << std::endl;
    std::cout << "#:1:phi\n#:2:theta\n#:3:psi\n#:4:alpha" << std::endl;
    
    
    for(int t=0; t<10*STEP_PER_SEC; t++) {
        std::cout << (double)t/STEP_PER_SEC << " ";
        
        const dReal* r = dBodyGetPosition(fBody1);
        const dReal* q = dBodyGetQuaternion(fBody1);
        const dReal* Q = dBodyGetRotation(fBody1);
        const dReal* v = dBodyGetLinearVel(fBody1);
        const dReal* omega = dBodyGetAngularVel(fBody1);
        
        const double theta = asin(-Q[8]);
        const double c_theta = cos(theta);
        
        const double phi = atan2(Q[4]/c_theta, Q[0]/c_theta);
        const double psi = atan2(Q[9]/c_theta, Q[10]/c_theta);
        
        const double alpha = dJointGetHingeAngle(fJoint2);
        
        std::cout << phi << " " << theta << " " << psi << " " << alpha << " ";
        
        std::cout << std::endl;
        
        for(int i=0; i<INT_PER_STEP; i++)
            dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
    }
    
    dWorldDestroy(fWorld);
}
