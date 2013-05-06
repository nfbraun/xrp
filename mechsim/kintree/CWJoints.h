#ifndef MSIM_CWJOINTS_H
#define MSIM_CWJOINTS_H

#include <Eigen/Dense>
#include "KinTree.h"

class KneeJoint: public Joint {
  public:
    KneeJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 1; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class InvKneeJoint: public Joint {
  public:
    InvKneeJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 1; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class AnkleJoint: public Joint {
  public:
    AnkleJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 2; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class InvAnkleJoint: public Joint {
  public:
    InvAnkleJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 2; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class HipJoint: public Joint {
  public:
    HipJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 3; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class InvHipJoint: public Joint {
  public:
    InvHipJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 3; }
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

#endif
