#include <iostream>
#include <ode/ode.h>

int main()
{
    const int STEP_PER_SEC = 25;
    const int INT_PER_STEP = 100;
    
    dMass mass;
    
    dInitODE();
    
    dWorldID fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., 0.);
    
    dBodyID fBody = dBodyCreate(fWorld);
    dBodySetPosition(fBody, 0., 0., 0.);
    dMassSetParameters(&mass, 1.,          // total mass
                              0., 0., 0.,  // center of gravity
                              3., 1., 2.,  // I_11, I_22, I_33
                              0., 0., 0.); // I_12, I_13, I_23
    dBodySetMass(fBody, &mass);
    
    dQuaternion q0 = { 1., 0., 0., 0. };
    
    dBodySetQuaternion(fBody, q0);
    dBodySetLinearVel(fBody, 0., 0., 0.);
    dBodySetAngularVel(fBody, 1., 0.5, 0.7);
    
    std::cout << "#:1:omega_0" << std::endl;
    std::cout << "#:2:omega_1" << std::endl;
    std::cout << "#:3:omega_2" << std::endl;
    std::cout << "#:4:phi" << std::endl;
    std::cout << "#:5:theta" << std::endl;
    std::cout << "#:6:psi" << std::endl;
    
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
        
        const double theta = asin(-Q[2]);
        const double c_theta = cos(theta);
        
        const double phi = atan2(Q[1]/c_theta, Q[0]/c_theta);
        const double psi = atan2(Q[6]/c_theta, Q[10]/c_theta);
        
        double T = omegaP[0] * 1. * omegaP[0]
                 + omegaP[1] * 1.5 * omegaP[1]
                 + omegaP[2] * .5 * omegaP[2];
                 
        //std::cout << r[0] << " " << r[1] << " " << r[2] << " ";
        //std::cout << v[0] << " " << v[1] << " " << v[2] << " ";
        std::cout << omegaP[0] << " " << omegaP[1] << " " << omegaP[2] << " ";
        //std::cout << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " ";
        std::cout << phi << " " << theta << " " << psi << " ";
        
        //std::cout << T << " ";
        
        std::cout << std::endl;
        
        for(int i=0; i<INT_PER_STEP; i++)
            dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
    }
    
    dWorldDestroy(fWorld);
}
