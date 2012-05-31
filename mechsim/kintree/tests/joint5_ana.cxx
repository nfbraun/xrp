#include <iostream>
#include <algorithm>
#include <cmath>
#include "RawODESolver.h"
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"
#include <gsl/gsl_errno.h>

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
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (-C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(-C - dVdq);
    
    return GSL_SUCCESS;
}

int main()
{
    struct JointSysData fData;
    
    RigidBody* body0 = new RigidBody(0, Eigen::Vector3d(0., 0., 0.), "body0");
    BallJoint* joint1 = new BallJoint(body0, Eigen::Vector3d(0., 0., 0.), "joint1");
    RigidBody* body1 = new RigidBody(joint1, Eigen::Vector3d(0., 0., -1.), "body1");
    body1->setMass(1., 1., .5, .7, 0., 0., 0.);
    
    HingeJoint* joint2 = new HingeJoint(body1, Eigen::Vector3d(0., 0., -1.5), "joint2");
    RigidBody* body2 = new RigidBody(joint2, Eigen::Vector3d(0., 0., -2.), "body2");
    body2->setMass(.5, .1, .2, .3, 0., 0., 0.);
    
    UniversalJoint* joint3 = new UniversalJoint(body2, Eigen::Vector3d(0., 0., -2.5), "joint3");
    RigidBody* body3 = new RigidBody(joint3, Eigen::Vector3d(0., 0., -3.), "body3");
    body3->setMass(.9, .5, .4, .3, 0., 0., 0.);
    
    KinTree kt(body0);
    kt.AssignParamIDs();
    kt.setG(Eigen::Vector3d(0., 0., -1.));
    
    const double x_ini[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, -.5,  .7, 0.0, 0.0, 0.0 };
    const double tstep = 1./16.;
    int i;
    
    fData.kt = &kt;
    fData.ndim = kt.npar();
    
    RawODESolver solver(2*kt.npar(), calc_xdot, 0, x_ini, &fData);
    
    std::cout << "#:1:phi_ana" << std::endl;
    std::cout << "#:2:theta_ana" << std::endl;
    std::cout << "#:3:psi_ana" << std::endl;
    std::cout << "#:4:alpha_ana" << std::endl;
    std::cout << "#:5:beta_ana" << std::endl;
    std::cout << "#:6:gamma_ana" << std::endl;
    
    for(i=0; i<(16*10); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << solver.y()[0] << " ";
        std::cout << solver.y()[1] << " ";
        std::cout << solver.y()[2] << " ";
        std::cout << solver.y()[3] << " ";
        std::cout << solver.y()[4] << " ";
        std::cout << solver.y()[5] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
