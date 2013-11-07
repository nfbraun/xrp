#include "Robot.h"

const unsigned int Robot::fNJntDof[N_JOINTS] = { 2, 1, 3, 3, 1, 2 };

const SpMot Robot::fS[N_DOF] = {
    -SpMot::UnitRotX(),   // LAX
    -SpMot::UnitRotY(),   // LAY
    -SpMot::UnitRotY(),   // LKY
    -SpMot::UnitRotX(),   // LHX
    -SpMot::UnitRotY(),   // LHY
    -SpMot::UnitRotZ(),   // LHZ
    
    SpMot::UnitRotZ(),    // RHZ
    SpMot::UnitRotY(),    // RHY
    SpMot::UnitRotX(),    // RHX
    SpMot::UnitRotY(),    // RKY
    SpMot::UnitRotY(),    // RAY
    SpMot::UnitRotX()     // RAX
};

const SpInertia Robot::fI[N_JOINTS] = {
    SpInertia(rbMass(B_L_SHANK),
              Eigen::Vector3d(0., 0., CharacterConst::shankSizeZ/2.),
              rbMOI(B_L_SHANK).asDiagonal()),
    SpInertia(rbMass(B_L_THIGH),
              Eigen::Vector3d(0., 0., CharacterConst::thighSizeZ/2.),
              rbMOI(B_L_THIGH).asDiagonal()),
    SpInertia(rbMass(B_PELVIS),
              Eigen::Vector3d(0., -CharacterConst::legPosY_L, CharacterConst::pelvisSizeZ/2.),
              rbMOI(B_PELVIS).asDiagonal()),
    SpInertia(rbMass(B_R_THIGH),
              Eigen::Vector3d(0., 0., -CharacterConst::thighSizeZ/2.),
              rbMOI(B_R_THIGH).asDiagonal()),
    SpInertia(rbMass(B_R_SHANK),
              Eigen::Vector3d(0., 0., -CharacterConst::shankSizeZ/2.),
              rbMOI(B_R_SHANK).asDiagonal()),
    SpInertia(rbMass(B_R_FOOT),
              Eigen::Vector3d(CharacterConst::footPosX, 0., -CharacterConst::footSizeZ/2.),
              rbMOI(B_R_FOOT).asDiagonal())
};

const SE3Tr Robot::fBodyT[N_JOINTS] = {
    SE3Tr::Trans(0., 0., CharacterConst::shankSizeZ),
    SE3Tr::Trans(0., 0., CharacterConst::thighSizeZ),
    SE3Tr::Trans(0., -CharacterConst::legPosY_L + CharacterConst::legPosY_R, 0.),
    SE3Tr::Trans(0., 0., -CharacterConst::thighSizeZ),
    SE3Tr::Trans(0., 0., -CharacterConst::shankSizeZ),
    SE3Tr::Identity()
};
