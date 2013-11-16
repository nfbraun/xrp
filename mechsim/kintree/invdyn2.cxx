#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "CSVFile.h"
#include "Robot.h"
#include "RobotLeg.h"
#include "RNE_CRB.h"

class ExtState {
  public:
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    Eigen::Vector3d vel;
    Eigen::Vector3d avel;
    
    Eigen::Vector3d lFtot, lTtot;
    Eigen::Vector3d rFtot, rTtot;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Vector3d readVector(const CSVFile& csv, unsigned int t_idx, const unsigned int idx[3])
{
    Eigen::Vector3d v;
    v.x() = csv.data().at(t_idx).at(idx[0]);
    v.y() = csv.data().at(t_idx).at(idx[1]);
    v.z() = csv.data().at(t_idx).at(idx[2]);
    
    return v;
}

Eigen::Quaterniond readQuaternion(const CSVFile& csv, unsigned int t_idx, const unsigned int idx[4])
{
    Eigen::Quaterniond q;
    q.w() = csv.data().at(t_idx).at(idx[0]);
    q.x() = csv.data().at(t_idx).at(idx[1]);
    q.y() = csv.data().at(t_idx).at(idx[2]);
    q.z() = csv.data().at(t_idx).at(idx[3]);
    
    return q;
}

ExtState readState(const CSVFile& csv, unsigned int t_idx)
{
    const unsigned int posIdx[3] = { 37, 38, 39 };     // FIXME
    const unsigned int rotIdx[4] = { 40, 41, 42, 43 }; // FIXME
    const unsigned int velIdx[3] = { 44, 45, 46 };     // FIXME
    const unsigned int avelIdx[3] = { 47, 48, 49 };    // FIXME
    const unsigned int lFIdx[3] = { 50, 51, 52 };      // FIXME
    const unsigned int lTIdx[3] = { 53, 54, 55 };      // FIXME
    const unsigned int rFIdx[3] = { 56, 57, 58 };      // FIXME
    const unsigned int rTIdx[3] = { 59, 60, 61 };      // FIXME
    
    ExtState estate;
    estate.pos = readVector(csv, t_idx, posIdx);
    estate.rot = readQuaternion(csv, t_idx, rotIdx);
    estate.vel = readVector(csv, t_idx, velIdx);
    estate.avel = readVector(csv, t_idx, avelIdx);
    
    estate.lFtot = readVector(csv, t_idx, lFIdx);
    estate.lTtot = readVector(csv, t_idx, lTIdx);
    estate.rFtot = readVector(csv, t_idx, rFIdx);
    estate.rTtot = readVector(csv, t_idx, rTIdx);
    
    return estate;
}

void printVect(const Eigen::Vector3d& v)
{
    std::cout << v.x() << " ";
    std::cout << v.y() << " ";
    std::cout << v.z() << " ";
}

int main()
{
    CSVFile csv("step.dat");
    if(csv.fail()) {
        throw std::runtime_error("Failed to open step.dat");
    }
    
    const unsigned int N_DOF = 2 * RobotLeg::N_DOF;
    const char* names[] = { "LHZ", "LHY", "LHX", "LKY", "LAY", "LAX", "RHZ", "RHY", "RHX", "RKY", "RAY", "RAX" };
    
    for(unsigned int i=0; i<N_DOF; i++)
        std::cout << "#:" << i+1 << ":p_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<N_DOF; i++)
        std::cout << "#:" << i+1+N_DOF << ":o_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<N_DOF; i++)
        std::cout << "#:" << i+1+2*N_DOF << ":t_" << names[i] << "_ana" << std::endl;
    
    unsigned int t_idx_end = csv.data().size() - 2;
    
    // Will fail for variable intervals!
    const double dt = csv.data().at(1).at(0) - csv.data().at(0).at(0);
    
    for(unsigned int t_idx=1; t_idx<t_idx_end; t_idx++) {
        double t = csv.data().at(t_idx).at(0);
        
        std::cout << t << " ";
        
        Eigen::VectorXd q(N_DOF);
        Eigen::VectorXd qdot(N_DOF);
        Eigen::VectorXd qddot(N_DOF);
        
        for(unsigned int dof_idx=0; dof_idx<N_DOF; dof_idx++) {
            double q_ = csv.data().at(t_idx).at(dof_idx+1);
            double q_plus = csv.data().at(t_idx+1).at(dof_idx+1);
            double q_minus = csv.data().at(t_idx-1).at(dof_idx+1);
            
            double qdot_ = csv.data().at(t_idx).at(dof_idx+1+N_DOF);
            double qdot_plus = csv.data().at(t_idx+1).at(dof_idx+1+N_DOF);
            double qdot_minus = csv.data().at(t_idx-1).at(dof_idx+1+N_DOF);
            
            q(dof_idx) = q_;
            qdot(dof_idx) = qdot_;
            qddot(dof_idx) = (qdot_plus - qdot_minus) / (2. * dt);
            //qddot(dof_idx) = (q_plus + q_minus - 2*q_) / (dt * dt);
        }
        
        ExtState estate = readState(csv, t_idx);
        ExtState estate_plus = readState(csv, t_idx+1);
        ExtState estate_minus = readState(csv, t_idx-1);
        
        const Eigen::Vector3d g(0., 0., -9.81);
        
        /* Transform pelvis velocity */
        SpMot v_p_ode(estate.avel, estate.vel);
        SpMot a_p_ode((estate_plus.avel - estate_minus.avel) / (2.*dt),
                      (estate_plus.vel - estate_minus.vel) / (2.*dt));
        
        SpMot v_p = v_p_ode.tr(SE3Tr::Rot(estate.rot).inverse());
        
        // FIXME: correct?
        a_p_ode += SpMot(Eigen::Vector3d::Zero(), -g);
        SpMot a_p = a_p_ode.tr(SE3Tr::Rot(estate.rot).inverse());
        a_p -= SpMot(v_p.ang(), Eigen::Vector3d::Zero()).cross(v_p);
        
        /* Transform reaction force on left foot */
        SE3Tr flrot = SE3Tr::Rot(estate.rot) *
            SE3Tr::RotZ(q(LHZ)) *
            SE3Tr::RotY(q(LHY)) *
            SE3Tr::RotX(q(LHX)) *
            SE3Tr::RotY(q(LKY)) *
            SE3Tr::RotY(q(LAY)) *
            SE3Tr::RotX(q(LAX));
        
        SpForce fl_ode(estate.lTtot, estate.lFtot);
        SpForce fl = fl_ode.tr(flrot.inverse());
        fl = fl.tr(SE3Tr::Trans(CharacterConst::footPosX, 0., -CharacterConst::footSizeZ/2.));
        
        /* Transform reaction force on right foot */
        SE3Tr frrot = SE3Tr::Rot(estate.rot) *
            SE3Tr::RotZ(q(RHZ)) *
            SE3Tr::RotY(q(RHY)) *
            SE3Tr::RotX(q(RHX)) *
            SE3Tr::RotY(q(RKY)) *
            SE3Tr::RotY(q(RAY)) *
            SE3Tr::RotX(q(RAX));
        
        SpForce fr_ode(estate.rTtot, estate.rFtot);
        SpForce fr = fr_ode.tr(frrot.inverse());
        fr = fr.tr(SE3Tr::Trans(CharacterConst::footPosX, 0., -CharacterConst::footSizeZ/2.));
        
        Eigen::VectorXd u(Robot::N_DOF);
        
        /* Calculate inverse dynamics for right leg */
        RobotLeg rl;
        Eigen::Matrix<SE3Tr, RobotLeg::N_DOF, 1> expSq_rleg =
            rl.calc_expSq(q.block<RobotLeg::N_DOF,1>(6,0));
        
        u.block<RobotLeg::N_DOF,1>(6,0) = calc_u_2(rl, expSq_rleg,
            qdot.block<RobotLeg::N_DOF,1>(6,0),
            qddot.block<RobotLeg::N_DOF,1>(6,0),
            v_p.tr(SE3Tr::Trans(0., CharacterConst::legPosY_R, -CharacterConst::pelvisSizeZ/2.).inverse()),
            a_p.tr(SE3Tr::Trans(0., CharacterConst::legPosY_R, -CharacterConst::pelvisSizeZ/2.).inverse()),
            -fr);
        
        /* Calculate inverse dynamics for left leg */
        Eigen::Matrix<SE3Tr, RobotLeg::N_DOF, 1> expSq_lleg =
            rl.calc_expSq(q.block<RobotLeg::N_DOF,1>(0,0));
        
        u.block<RobotLeg::N_DOF,1>(0,0) = calc_u_2(rl, expSq_lleg,
            qdot.block<RobotLeg::N_DOF,1>(0,0),
            qddot.block<RobotLeg::N_DOF,1>(0,0),
            v_p.tr(SE3Tr::Trans(0., CharacterConst::legPosY_L, -CharacterConst::pelvisSizeZ/2.).inverse()),
            a_p.tr(SE3Tr::Trans(0., CharacterConst::legPosY_L, -CharacterConst::pelvisSizeZ/2.).inverse()),
            -fl);
        
        /* Print results */
        for(unsigned int dof_idx=0; dof_idx<N_DOF; dof_idx++)
            std::cout << q(dof_idx) << " ";
        for(unsigned int dof_idx=0; dof_idx<N_DOF; dof_idx++)
            std::cout << qdot(dof_idx) << " ";
        for(unsigned int dof_idx=0; dof_idx<N_DOF; dof_idx++)
            std::cout << u(dof_idx) << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}
