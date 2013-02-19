#include <iostream>
#include "Cartwheel.h"
#include "DynTransform.h"

inline double sqr(double x)
{
    return x*x;
}

void compareFullStates(const FullState& s1, const FullState& s2)
{
    double pos_mse = 0.;
    double rot_mse = 0.;
    double vel_mse = 0.;
    double avel_mse = 0.;
    
    for(unsigned int i=0; i<B_MAX; i++) {
        pos_mse += (s1.pos(i) - s2.pos(i)).squaredNorm();
        rot_mse += (s1.rot(i).toRotationMatrix() - s2.rot(i).toRotationMatrix()).squaredNorm();
        vel_mse += (s1.vel(i) - s2.vel(i)).squaredNorm();
        avel_mse += (s1.avel(i) - s2.avel(i)).squaredNorm();
    }
    
    std::cout << sqrt(pos_mse/B_MAX) << " " << sqrt(rot_mse/B_MAX) << " ";
    std::cout << sqrt(vel_mse/B_MAX) << " " << sqrt(avel_mse/B_MAX) << " ";
}

void test_leg_constraint(unsigned int side, const FullState& fstate, double& c_pos_mse, double& c_vel_mse)
{
    using namespace CharacterConst;
    
    Eigen::Vector3d rHipPos1;
    if(side == LEFT) {
        rHipPos1 = fstate.pos(B_PELVIS) + 
            fstate.rot(B_PELVIS)._transformVector(Eigen::Vector3d(0., legPosY_L, -pelvisSizeZ/2.));
    } else {
        rHipPos1 = fstate.pos(B_PELVIS) + 
            fstate.rot(B_PELVIS)._transformVector(Eigen::Vector3d(0., legPosY_R, -pelvisSizeZ/2.));
    }
    Eigen::Vector3d rHipPos2 = fstate.pos(side, B_THIGH) + 
        fstate.rot(side, B_THIGH)._transformVector(Eigen::Vector3d(0., 0., thighSizeZ/2.));
    
    c_pos_mse += (rHipPos1 - rHipPos2).squaredNorm();
    
    Eigen::Vector3d rHipVel1 = fstate.vel(B_PELVIS) + 
        fstate.avel(B_PELVIS).cross(rHipPos1 - fstate.pos(B_PELVIS));
    Eigen::Vector3d rHipVel2 = fstate.vel(side, B_THIGH) + 
        fstate.avel(side, B_THIGH).cross(rHipPos2 - fstate.pos(side, B_THIGH));
    
    c_vel_mse += (rHipVel1 - rHipVel2).squaredNorm();
    
    Eigen::Vector3d rKneePos1 = fstate.pos(side, B_THIGH) + 
        fstate.rot(side, B_THIGH)._transformVector(Eigen::Vector3d(0., 0., -thighSizeZ/2.));
    Eigen::Vector3d rKneePos2 = fstate.pos(side, B_SHANK) + 
        fstate.rot(side, B_SHANK)._transformVector(Eigen::Vector3d(0., 0., shankSizeZ/2.));
    
    c_pos_mse += (rKneePos1 - rKneePos2).squaredNorm();
    
    Eigen::Vector3d rKneeVel1 = fstate.vel(side, B_THIGH) + 
        fstate.avel(side, B_THIGH).cross(rKneePos1 - fstate.pos(side, B_THIGH));
    Eigen::Vector3d rKneeVel2 = fstate.vel(side, B_SHANK) + 
        fstate.avel(side, B_SHANK).cross(rKneePos2 - fstate.pos(side, B_SHANK));
    
    c_vel_mse += (rKneeVel1 - rKneeVel2).squaredNorm();
    
    Eigen::Vector3d rAnklePos1 = fstate.pos(side, B_SHANK) + 
        fstate.rot(side, B_SHANK)._transformVector(Eigen::Vector3d(0., 0., -shankSizeZ/2.));
    Eigen::Vector3d rAnklePos2 = fstate.pos(side, B_FOOT) + 
        fstate.rot(side, B_FOOT)._transformVector(Eigen::Vector3d(-0.016, 0., footSizeZ/2.));
    
    c_pos_mse += (rAnklePos1 - rAnklePos2).squaredNorm();
    
    Eigen::Vector3d rAnkleVel1 = fstate.vel(side, B_SHANK) + 
        fstate.avel(side, B_SHANK).cross(rAnklePos1 - fstate.pos(side, B_SHANK));
    Eigen::Vector3d rAnkleVel2 = fstate.vel(side, B_FOOT) + 
        fstate.avel(side, B_FOOT).cross(rAnklePos2 - fstate.pos(side, B_FOOT));
    
    c_vel_mse += (rAnkleVel1 - rAnkleVel2).squaredNorm();
}

void test_constraint(const FullState& fstate)
{
    double c_pos_mse = 0.;
    double c_vel_mse = 0.;

    test_leg_constraint(LEFT, fstate, c_pos_mse, c_vel_mse);
    test_leg_constraint(RIGHT, fstate, c_pos_mse, c_vel_mse);
    
    std::cout << sqrt(c_pos_mse/6.) << " " << sqrt(c_vel_mse/6.) << " ";
}

void test_joint_constraint(const FullState& fstate)
{
    double knee_mse = 0.;
    double ankle_mse = 0.;
    
    Eigen::Vector3d ey_LT = fstate.rot(B_L_THIGH)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ey_LS = fstate.rot(B_L_SHANK)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ex_LF = fstate.rot(B_L_FOOT)._transformVector(Eigen::Vector3d::UnitX());
    
    knee_mse += (ey_LT - ey_LS).squaredNorm();
    ankle_mse += sqr(ey_LS.dot(ex_LF));
    
    Eigen::Vector3d ey_RT = fstate.rot(B_R_THIGH)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ey_RS = fstate.rot(B_R_SHANK)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ex_RF = fstate.rot(B_R_FOOT)._transformVector(Eigen::Vector3d::UnitX());
    
    knee_mse += (ey_RT - ey_RS).squaredNorm();
    ankle_mse += sqr(ey_RS.dot(ex_RF));
    
    std::cout << sqrt(knee_mse/2.) << " " << sqrt(ankle_mse/2.) << " ";
}

int main()
{
    Cartwheel sim(2500, 1);
    
    // Various constraint violations
    std::cout << "#:1:c_pos_mse" << std::endl;
    std::cout << "#:2:c_vel_mse" << std::endl;
    std::cout << "#:3:knee_mse" << std::endl;
    std::cout << "#:4:ankle_mse" << std::endl;
    
    // Reprojection error
    std::cout << "#:5:pos_mse" << std::endl;
    std::cout << "#:6:rot_mse" << std::endl;
    std::cout << "#:7:vel_mse" << std::endl;
    std::cout << "#:8:avel_mse" << std::endl;
    
    for(int t=0; t<10000; t++) {
        sim.Advance();
        const CartState state = sim.GetCurrentState();
        
        std::cout << t*sim.GetTimestep() << " ";
        test_constraint(state.fFState);
        test_joint_constraint(state.fFState);
        
        FullState fstate = fullFromJoint(jointFromFull(state.fFState));
        setBodyState(fstate, B_L_FOOT, state.fFState.q(B_L_FOOT));
        compareFullStates(state.fFState, fstate);
        
        std::cout << std::endl;
    }
    
    return 0;
}
