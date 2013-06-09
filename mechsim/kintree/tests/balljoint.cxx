#include <iostream>
#include <Eigen/Dense>
#include <ode/ode.h>

void decompZYXRot(const Eigen::Quaterniond q, double& z, double& y, double &x)
{
    const double R00 = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
    const double R10 = 2.*q.x()*q.y() + 2.*q.w()*q.z();
    const double R20 = 2.*q.x()*q.z() - 2.*q.w()*q.y();
    const double R21 = 2.*q.y()*q.z() + 2.*q.w()*q.x();
    const double R22 = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
    
    z = atan2(R10, R00);
    y = asin(-R20);
    x = atan2(R21, R22);
}

// Transform hip torque from generalized forces to torque in Cartesian coordinates
Eigen::Vector3d transformHipTorque(double hz, double hy, double hx, double thz, double thy, double thx)
{
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d QinvT;
    
    QinvT << cz/cy, -sz, sy*cz/cy,
             sz/cy,  cz, sy*sz/cy,
             0.,     0.,       1.;
    
    return QinvT * Eigen::Vector3d(thx, thy, thz);
}

// Transform hip angular velocity from angular velocity in Cartesian coordinates to generalized velocities
void invHipAVelTransform(double hz, double hy, double hx, Eigen::Vector3d& avel, double& ohz, double& ohy, double& ohx)
{
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d Qinv;
    
    Qinv << cz/cy,    sz/cy,    0.,
            -sz,      cz,       0.,
            sy*cz/cy, sy*sz/cy, 1.;
    
    ohx = (Qinv*avel)(0);
    ohy = (Qinv*avel)(1);
    ohz = (Qinv*avel)(2);
}

int main()
{
    const int STEP_PER_SEC = 250;
    const int INT_PER_STEP = 10;
    
    dMass mass;
    
    dInitODE();
    
    dWorldID fWorld = dWorldCreate();
    dWorldSetGravity(fWorld, 0., 0., 1.);
    
    dBodyID fBody = dBodyCreate(fWorld);
    dBodySetPosition(fBody, 0., 0., 1.);
    dMassSetParameters(&mass, 1.,          // total mass
                              0., 0., 0.,  // center of gravity
                              3., 1., 2.,  // I_11, I_22, I_33
                              0., 0., 0.); // I_12, I_13, I_23
    dBodySetMass(fBody, &mass);
    
    dQuaternion q0 = { 1., 0., 0., 0. };
    
    dBodySetQuaternion(fBody, q0);
    dBodySetLinearVel(fBody, 0., 0., 0.);
    dBodySetAngularVel(fBody, 0., 0., 1.);
    
    dJointID fJoint = dJointCreateBall(fWorld, 0);
    dJointAttach(fJoint, 0, fBody);
    dJointSetBallAnchor(fJoint, 0., 0., 0.);
    
    std::cout << "#:1:HZ" << std::endl;
    std::cout << "#:2:HY" << std::endl;
    std::cout << "#:3:HX" << std::endl;
    std::cout << "#:4:OZ" << std::endl;
    std::cout << "#:5:OY" << std::endl;
    std::cout << "#:6:OX" << std::endl;
    
    for(int t=0; t<10*STEP_PER_SEC; t++) {
        std::cout << (double)t/STEP_PER_SEC << " ";
        
        const dReal* r = dBodyGetPosition(fBody);
        const dReal* q_raw = dBodyGetQuaternion(fBody);
        const dReal* Q = dBodyGetRotation(fBody);
        const dReal* v = dBodyGetLinearVel(fBody);
        const dReal* avel_raw = dBodyGetAngularVel(fBody);
        
        Eigen::Quaterniond q(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);
        Eigen::Vector3d avel(avel_raw[0], avel_raw[1], avel_raw[2]);
        
        double hz, hy, hx;
        
        decompZYXRot(q, hz, hy, hx);
        
        std::cout << hz << " " << hy << " " << hx << " ";
        
        double ohz, ohy, ohx;
        invHipAVelTransform(hz, hy, hx, avel, ohz, ohy, ohx);
        
        std::cout << ohz << " " << ohy << " " << ohx << " ";
        std::cout << std::endl;
        
        for(int i=0; i<INT_PER_STEP; i++) {
            q_raw = dBodyGetQuaternion(fBody);
            q = Eigen::Quaterniond(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);
            
            decompZYXRot(q, hz, hy, hx);
            
            // Eigen::Vector3d T = transformHipTorque(hz, hy, hx, 0.5, 0.3, 1.0);
            Eigen::Vector3d T(0.5, 0.3, 1.0);
            
            dBodyAddTorque(fBody, T.x(), T.y(), T.z());
            
            dWorldStep(fWorld, 1./(STEP_PER_SEC * INT_PER_STEP));
        }
    }
    
    dWorldDestroy(fWorld);
}
