#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RawODESolver.h"
#include <cassert>
#include <Eigen/Dense>
#include "../KinTree.h"
#include "../CWJoints.h"
#include "../../cartwheel_walker/StaticRobotInfo.h"
#include <gsl/gsl_errno.h>

// Transform hip torque from torque in Cartesian coordinates to generalized forces
void invTransformHipTorque(double hz, double hy, double hx, const Eigen::Vector3d& T, double& thz, double& thy, double& thx)
{
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    thx = cy*cz*T.x() + cy*sz*T.y() - sy*T.z();
    thy = -sz*T.x() + cz*T.y();
    thz = T.z();
}

int n_xdot_eval = 0;

struct JointSysData {
    KinTree* kt;
    unsigned int ndim;
};

int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    struct JointSysData* data = (struct JointSysData*) _p;
    Eigen::Map<const Eigen::VectorXd> q(x_raw, data->ndim);
    Eigen::Map<const Eigen::VectorXd> qdot(x_raw+data->ndim, data->ndim);
    
    data->kt->calcDyn(q, qdot);
    Eigen::MatrixXd M = data->kt->M();
    
    Eigen::VectorXd C = data->kt->C();
    
    Eigen::VectorXd dVdq = data->kt->dVdq();
    
    double thz, thy, thx;
    Eigen::Vector3d T(0.5, 0.3, 1.0);
    invTransformHipTorque(q(0), q(1), q(2), T, thz, thy, thx);
    
    Eigen::VectorXd u(data->kt->npar());
    u.setZero();
    
    u(0) = thz;
    u(1) = thy;
    u(2) = thx;
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (u - C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(u - C - dVdq);
    
    n_xdot_eval++;
    
    return GSL_SUCCESS;
}

RigidBody* construct()
{
    using namespace CharacterConst;
    using Eigen::Vector3d;
    
    RigidBody* fix = new RigidBody(0, Vector3d(0., 0., 0.), "torso");
    fix->setMass(0., Eigen::Vector3d(0., 0., 0.));
    
    Joint* joint = new HipJoint(fix, Vector3d(0., 0., 0.), "joint");
    RigidBody* body = new RigidBody(joint, Vector3d(0., 0., 1.), "body");
    body->setMass(1., Eigen::Vector3d(3., 1., 2.));
    
    return fix;
}

double normalizeAngle(double x)
{
    if(x > 0)
        return fmod(x + M_PI, 2*M_PI) - M_PI;
    else
        return fmod(x - M_PI, 2*M_PI) + M_PI;
}

int main()
{
    struct JointSysData fData;
    
    KinTree kt(construct());
    kt.AssignParamIDs();
    kt.setG(Eigen::Vector3d(0., 0., 1.));
    
    std::vector<double> x_ini(2*kt.npar(), 0.0);
    x_ini.at(3) = 1.0;
    const double tstep = 1./160.;
    int i;
    
    fData.kt = &kt;
    fData.ndim = kt.npar();
    
    RawODESolver solver(2*kt.npar(), calc_xdot, 0, x_ini.data(), &fData);
    
    const char* names[] = { "HZ", "HY", "HX" };
    
    assert(kt.npar() == 3);
    
    for(unsigned int i=0; i<kt.npar(); i++)
        std::cout << "#:" << i+1 << ":p_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<kt.npar(); i++)
        std::cout << "#:" << i+1+kt.npar() << ":o_" << names[i] << "_ana" << std::endl;
    
    for(i=0; i<10*160; ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        
        for(unsigned int i=0; i<kt.npar(); i++)
            std::cout << normalizeAngle(solver.y()[i]) << " ";
        for(unsigned int i=0; i<kt.npar(); i++)
            std::cout << solver.y()[i+kt.npar()] << " ";
        std::cout << std::endl;
    }
    
    std::cerr << "n_xdot_eval: " << n_xdot_eval << std::endl;
    
    return 0;
}
