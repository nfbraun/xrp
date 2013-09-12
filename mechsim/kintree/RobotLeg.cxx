#include "RobotLeg.h"

const unsigned int RobotLeg::fNJntDof[N_JOINTS] = { 3, 1, 2 };

const SpMot RobotLeg::fS[N_DOF] = {
    SpMot(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 0.)),
    SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
    SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.)),
    SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
    SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
    SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.))
};

const SpInertia RobotLeg::fI[N_JOINTS] = {
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

const SE3Tr RobotLeg::fBodyT[N_JOINTS] = {
    SE3Tr::Trans(0., 0., -CharacterConst::thighSizeZ),
    SE3Tr::Trans(0., 0., -CharacterConst::shankSizeZ),
    SE3Tr::Identity()
};
