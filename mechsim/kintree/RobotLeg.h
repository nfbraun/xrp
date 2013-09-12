#ifndef MSIM_ROBOTLEG_H
#define MSIM_ROBOTLEG_H

#include "KinChain2.h"
#include "../mechsim/cartwheel_walker/StaticRobotInfo.h"

class RobotLeg: public KinChain2 {
  public:
    static const unsigned int N_DOF = 6;
    static const unsigned int N_JOINTS = 3;
    
    virtual unsigned int nJoints() const { return N_JOINTS; }
    
    virtual unsigned int nDof() const { return N_DOF; }
    
    virtual unsigned int nJntDof(unsigned int i) const
    {
        assert(i < 3);
        return fNJntDof[i];
    }
    
    virtual SpMot S(unsigned int dof_idx) const
    {
        assert(dof_idx < 6);
        return fS[dof_idx];
    }
    
    virtual SpInertia I(unsigned int idx) const
    {
        assert(idx < 3);
        return fI[idx];
    }
    
    virtual SE3Tr bodyT(unsigned int body_id) const
    {
        assert(body_id < 3);
        return fBodyT[body_id];
    }
    
    Eigen::Matrix<SE3Tr, N_DOF, 1> calc_expSq(const Eigen::VectorXd& q)
    {
        Eigen::Matrix<SE3Tr, N_DOF, 1> expSq;
        
        expSq(0) = SE3Tr::RotZ(q(0));
        expSq(1) = SE3Tr::RotY(q(1));
        expSq(2) = SE3Tr::RotX(q(2));
        expSq(3) = SE3Tr::RotY(q(3));
        expSq(4) = SE3Tr::RotY(q(4));
        expSq(5) = SE3Tr::RotX(q(5));
        
        return expSq;
    }
    
    static const unsigned int fNJntDof[N_JOINTS];
    static const SpMot fS[N_DOF];
    static const SpInertia fI[N_JOINTS];
    static const SE3Tr fBodyT[N_JOINTS];
};

#endif
