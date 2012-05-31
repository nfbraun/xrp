#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RawODESolver.h"
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"
#include <gsl/gsl_errno.h>

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
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (-C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(-C - dVdq);
    
    n_xdot_eval++;
    
    return GSL_SUCCESS;
}

namespace CharacterConst {
    const double density = 900;
    
    const double footSizeX = 0.12;
    const double footSizeY = 0.2;
    const double footSizeZ = 0.05;
    const double legDiameter = 0.1;
    const double legSizeZ = 1.0;
    const double kneeRelativePosZ = 0.5;
    const double legRelativeAnchorX = 0.6;
    
    const double torsoSizeX = 0.45;
    const double torsoSizeY = 0.25;
    const double torsoSizeZ = 0.6;
    
    const double shankSizeZ = legSizeZ * kneeRelativePosZ;
    const double thighSizeZ = legSizeZ - shankSizeZ;
    
    const double footPosZ = footSizeZ/2.;
    const double anklePosZ = footSizeZ;
    const double shankPosZ = anklePosZ + shankSizeZ/2.;
    const double kneePosZ = anklePosZ + legSizeZ * kneeRelativePosZ;
    const double thighPosZ = kneePosZ + thighSizeZ/2.;
    const double hipPosZ = anklePosZ + legSizeZ;
    const double torsoPosZ = hipPosZ + torsoSizeZ/2.;
    
    const double legPosX_L = torsoSizeX/2.0*legRelativeAnchorX;
    const double legPosX_R = -torsoSizeX/2.0*legRelativeAnchorX;
    
    const double footPosY = -0.016;
} // end namespace CharacterConst

RigidBody* construct()
{
    using namespace CharacterConst;
    using Eigen::Vector3d;
    
    RigidBody* lFoot = new RigidBody(0, Vector3d(legPosX_L, footPosY, footPosZ), "lFoot");
    lFoot->setMassBox(density, footSizeX, footSizeY, footSizeZ);
    
    Joint* lAnkle = new UniversalJoint(lFoot, Vector3d(legPosX_L, 0, anklePosZ), "lAnkle");
    
    RigidBody* lShank = new RigidBody(lAnkle, Vector3d(legPosX_L, 0, shankPosZ), "lShank");
    lShank->setMassCylinder(density, shankSizeZ, legDiameter/2.);
    
    Joint* lKnee = new HingeJoint(lShank, Vector3d(legPosX_L, 0, kneePosZ), "lKnee");
    
    RigidBody* lThigh = new RigidBody(lKnee, Vector3d(legPosX_L, 0, thighPosZ), "lThigh");
    lThigh->setMassCylinder(density, thighSizeZ, legDiameter/2.);
    
    Joint* lHip = new BallJoint(lThigh, Vector3d(legPosX_L, 0, hipPosZ), "lHip");
    
    RigidBody* torso = new RigidBody(lHip, Vector3d(0, 0, torsoPosZ), "torso");
    torso->setMassBox(0.5*density, torsoSizeX, torsoSizeY, torsoSizeZ);
    
    Joint* rHip = new BallJoint(torso, Vector3d(legPosX_R, 0, hipPosZ), "rHip");
    RigidBody* rThigh = new RigidBody(rHip, Vector3d(legPosX_R, 0, thighPosZ), "rThigh");
    rThigh->setMassCylinder(density, thighSizeZ, legDiameter/2.);
    
    Joint* rKnee = new HingeJoint(rThigh, Vector3d(legPosX_R, 0, kneePosZ), "rKnee");
    RigidBody* rShank = new RigidBody(rKnee, Vector3d(legPosX_R, 0, shankPosZ), "rShank");
    rShank->setMassCylinder(density, shankSizeZ, legDiameter/2.);
    
    Joint* rAnkle = new UniversalJoint(rShank, Vector3d(legPosX_R, 0, anklePosZ), "rAnkle");
    RigidBody* rFoot = new RigidBody(rAnkle, Vector3d(legPosX_R, footPosY, footPosZ), "rFoot");
    rFoot->setMassBox(density, footSizeX, footSizeY, footSizeZ);
    
    /* std::cout << torso->pos() << std::endl;
    std::cout << lThigh->pos() << std::endl;
    std::cout << lShank->pos() << std::endl;
    std::cout << lFoot->pos() << std::endl; */
    /* std::cout << lHip->pos() << std::endl;
    std::cout << lKnee->pos() << std::endl;
    std::cout << lAnkle->pos() << std::endl; */
    
    return lThigh;
}

int main()
{
    struct JointSysData fData;
    
    KinTree kt(construct());
    kt.AssignParamIDs();
    kt.setG(Eigen::Vector3d(0., 0., -1.));
    
    std::vector<double> x_ini(2*kt.npar(), 0.0);
    const double tstep = 1./16.;
    int i;
    
    fData.kt = &kt;
    fData.ndim = kt.npar();
    
    RawODESolver solver(2*kt.npar(), calc_xdot, 0, x_ini.data(), &fData);
    
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
    
    std::cerr << n_xdot_eval << std::endl;
    
    return 0;
}
