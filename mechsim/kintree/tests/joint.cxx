#include <iostream>
#include <ode/ode.h>

int main()
{
    const int STEP_PER_SEC = 25;
    const int INT_PER_STEP = 100;
    
    const double I00 = 1., I11 = .5, I22 = .7;
    
    dMass mass;
    
    dInitODE();
    
    dWorldID fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., -1.);
    
    dBodyID fBody = dBodyCreate(fWorld);
    dBodySetPosition(fBody, 0., 0., -1.);
    dMassSetParameters(&mass, 1.,            // total mass
                              0., 0., 0.,    // center of gravity
                              I00, I11, I22, // I_11, I_22, I_33
                              0., 0., 0.);   // I_12, I_13, I_23
    dBodySetMass(fBody, &mass);
    
    dJointID fJoint = dJointCreateUniversal(fWorld, 0);
    dJointAttach(fJoint, fBody, 0);
    dJointSetUniversalAnchor(fJoint, 0., 0., 0.);
    dJointSetUniversalAxis1(fJoint, 1., 0., 0.);
    dJointSetUniversalAxis2(fJoint, 0., 1., 0.);
    
    const double phi_0 = .7;
    const double theta_0 = -.5;
    
    /* dQuaternion q0 = { 1., 0., 0., 0. };
    dBodySetQuaternion(fBody, q0); */
    dBodySetLinearVel(fBody, -theta_0, phi_0, 0.);
    dBodySetAngularVel(fBody, phi_0, theta_0, 0.);
    
    //std::cout << "#:1:x\n#:2:y\n#:3:z\n";
    //std::cout << "#:4:x_p\n#:5:y_p\n#:6:z_p" << std::endl;
    std::cout << "#:1:phi\n#:2:theta" << std::endl;
    
    
    for(int t=0; t<10*STEP_PER_SEC; t++) {
        std::cout << (double)t/STEP_PER_SEC << " ";
        
        const dReal* r = dBodyGetPosition(fBody);
        const dReal* q = dBodyGetQuaternion(fBody);
        const dReal* Q = dBodyGetRotation(fBody);
        const dReal* v = dBodyGetLinearVel(fBody);
        const dReal* omega = dBodyGetAngularVel(fBody);
        
        double omegaP[3] = { Q[0] * omega[0] + Q[4] * omega[1] + Q[8] * omega[2],
                             Q[1] * omega[0] + Q[5] * omega[1] + Q[9] * omega[2],
                             Q[2] * omega[0] + Q[6] * omega[1] + Q[10] * omega[2] };
        
        double T_rot = (omegaP[0] * I00 * omegaP[0]
                        + omegaP[1] * I11 * omegaP[1]
                        + omegaP[2] * I22 * omegaP[2])/2.;
        double T_tra = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2])/2.;
        
        double T = T_tra + T_rot;
        
        double V = r[2];
        
        dReal phi = dJointGetUniversalAngle1(fJoint);
        dReal theta = dJointGetUniversalAngle2(fJoint);
        
        dReal phi_dot = dJointGetUniversalAngle1Rate(fJoint);
        dReal theta_dot = dJointGetUniversalAngle2Rate(fJoint);
        
        double omega_test[3] = { phi_dot,
                                 theta_dot*cos(phi),
                                 -theta_dot*sin(phi) };
        
        dReal x_p = -sin(theta) * cos(phi);
        dReal y_p = sin(phi);
        dReal z_p = -cos(theta) * cos(phi);
        
        const double thetap = asin(-Q[8]);
        const double c_thetap = cos(theta);
        
        const double phip = atan2(Q[4]/c_thetap, Q[0]/c_thetap);
        const double psip = atan2(Q[9]/c_thetap, Q[10]/c_thetap);
        
        //std::cout << r[0] << " " << r[1] << " " << r[2] << " ";
        std::cout << phi << " " << theta << " ";
        std::cout << phip << " " << thetap << " " << psip << " ";
        
        //std::cout << T+V << " ";
        
        //std::cout << omegaP[0] << " " << omegaP[1] << " " << omegaP[2] << " ";
        //std::cout << omega_test[0] << " " << omega_test[1] << " " << omega_test[2] << " ";
        //std::cout << phi_dot << " " << theta_dot << " ";
        //std::cout << x_p << " " << y_p << " " << z_p << " ";
        
        std::cout << std::endl;
        
        for(int i=0; i<INT_PER_STEP; i++)
            dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
    }
    
    dWorldDestroy(fWorld);
}
