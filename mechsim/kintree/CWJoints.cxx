#include "CWJoints.h"
#include <stdexcept>

Eigen::Matrix4d KneeJoint::S(const Eigen::VectorXd& par) const
{
    return S_y(par[pid(0)]);
}

Eigen::Matrix4d InvKneeJoint::S(const Eigen::VectorXd& par) const
{
    return S_y(-par[pid(0)]);
}

Eigen::Matrix4d AnkleJoint::S(const Eigen::VectorXd& par) const
{
    return S_y(par[pid(0)]) * S_x(par[pid(1)]);
}

Eigen::Matrix4d InvAnkleJoint::S(const Eigen::VectorXd& par) const
{
    return S_x(-par[pid(0)]) * S_y(-par[pid(1)]);
}

Eigen::Matrix4d HipJoint::S(const Eigen::VectorXd& par) const
{
    return S_z(par[pid(0)]) * S_y(par[pid(1)]) * S_x(par[pid(2)]);
}

Eigen::Matrix4d InvHipJoint::S(const Eigen::VectorXd& par) const
{
    return S_x(-par[pid(0)]) * S_y(-par[pid(1)]) * S_z(-par[pid(2)]);
}

Eigen::Matrix4d KneeJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_y_dp(par[pid(0)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d InvKneeJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return -dS_y_dp(-par[pid(0)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d AnkleJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_y_dp(par[pid(0)]) * S_x(par[pid(1)]);
        case 1: return S_y(par[pid(0)]) * dS_x_dp(par[pid(1)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d InvAnkleJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return -dS_x_dp(-par[pid(0)]) * S_y(-par[pid(1)]);
        case 1: return -S_x(-par[pid(0)]) * dS_y_dp(-par[pid(1)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d HipJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_z_dp(par[pid(0)]) * S_y(par[pid(1)]) * S_x(par[pid(2)]);
        case 1: return S_z(par[pid(0)]) * dS_y_dp(par[pid(1)]) * S_x(par[pid(2)]);
        case 2: return S_z(par[pid(0)]) * S_y(par[pid(1)]) * dS_x_dp(par[pid(2)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d InvHipJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return -dS_x_dp(-par[pid(0)]) * S_y(-par[pid(1)]) * S_z(-par[pid(2)]);
        case 1: return -S_x(-par[pid(0)]) * dS_y_dp(-par[pid(1)]) * S_z(-par[pid(2)]);
        case 2: return -S_x(-par[pid(0)]) * S_y(-par[pid(1)]) * dS_z_dp(-par[pid(2)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d KneeJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 0 || j > 0) throw std::runtime_error("ParameterID out of range");
    return d2S_y_dpp(par[pid(0)]);
}

Eigen::Matrix4d InvKneeJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 0 || j > 0) throw std::runtime_error("ParameterID out of range");
    return d2S_y_dpp(-par[pid(0)]);
}

Eigen::Matrix4d AnkleJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 1 || j > 1) throw std::runtime_error("ParameterID out of range");
    switch(2*i+j) {
        case 0:         return d2S_y_dpp(par[pid(0)]) * S_x(par[pid(1)]);
        case 1: case 2: return dS_y_dp(par[pid(0)]) * dS_x_dp(par[pid(1)]);
        case 3:         return S_y(par[pid(0)]) * d2S_x_dpp(par[pid(1)]);
    }
}

Eigen::Matrix4d InvAnkleJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 1 || j > 1) throw std::runtime_error("ParameterID out of range");
    switch(2*i+j) {
        case 0:         return d2S_x_dpp(-par[pid(0)]) * S_y(-par[pid(1)]);
        case 1: case 2: return dS_x_dp(-par[pid(0)]) * dS_y_dp(-par[pid(1)]);
        case 3:         return S_x(-par[pid(0)]) * d2S_y_dpp(-par[pid(1)]);
    }
}

Eigen::Matrix4d HipJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 2 || j > 2) throw std::runtime_error("ParameterID out of range");
    switch(3*i+j) {
        case 0:         return d2S_z_dpp(par[pid(0)]) * S_y(par[pid(1)]) * S_x(par[pid(2)]);
        case 1: case 3: return dS_z_dp(par[pid(0)]) * dS_y_dp(par[pid(1)]) * S_x(par[pid(2)]);
        case 2: case 6: return dS_z_dp(par[pid(0)]) * S_y(par[pid(1)]) * dS_x_dp(par[pid(2)]);
        case 4:         return S_z(par[pid(0)]) * d2S_y_dpp(par[pid(1)]) * S_x(par[pid(2)]);
        case 5: case 7: return S_z(par[pid(0)]) * dS_y_dp(par[pid(1)]) * dS_x_dp(par[pid(2)]);
        case 8:         return S_z(par[pid(0)]) * S_y(par[pid(1)]) * d2S_x_dpp(par[pid(2)]);
    }
}

Eigen::Matrix4d InvHipJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 2 || j > 2) throw std::runtime_error("ParameterID out of range");
    switch(3*i+j) {
        case 0:         return d2S_x_dpp(-par[pid(0)]) * S_y(-par[pid(1)]) * S_z(-par[pid(2)]);
        case 1: case 3: return dS_x_dp(-par[pid(0)]) * dS_y_dp(-par[pid(1)]) * S_z(-par[pid(2)]);
        case 2: case 6: return dS_x_dp(-par[pid(0)]) * S_y(-par[pid(1)]) * dS_z_dp(-par[pid(2)]);
        case 4:         return S_x(-par[pid(0)]) * d2S_y_dpp(-par[pid(1)]) * S_z(-par[pid(2)]);
        case 5: case 7: return S_x(-par[pid(0)]) * dS_y_dp(-par[pid(1)]) * dS_z_dp(-par[pid(2)]);
        case 8:         return S_x(-par[pid(0)]) * S_y(-par[pid(1)]) * d2S_z_dpp(-par[pid(2)]);
    }
}

