#ifndef MSIM_KINTREE_H
#define MSIM_KINTREE_H

#include <string>
#include <list>
#include <stdexcept>
#include <Eigen/Dense>

#if 0
#include <ginac/ginac.h>
#endif

class Joint;

class RigidBody {
  friend class Joint;
  public:
    /* RigidBody(double m, double Ixx, double Iyy, double Izz, const std::string& name = std::string())
        : fm(m), fIxx(Ixx), fIyy(Iyy), fIzz(Izz), fName(name) {} */
    RigidBody(Joint* pJoint, const Eigen::Vector3d& pos, const std::string& name);
        
    const std::string& name() const { return fName; }
    Joint* pJoint() const { return fPJoint; }
    const std::list<Joint*>& cJoints() const { return fCJoints; }
    
    const Eigen::Vector3d& pos() const { return fPos; }
    
    void setMass(double m, double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz);
    void setMass(double m, const Eigen::Vector3d& Idiag);
    
    void setMassBox(double density, double sx, double sy, double sz);
    void setMassCylinder(double density, double h, double r);
    
    double m() const { return fm; }
    
    // inertia tensor about the center of gravity
    const Eigen::Matrix3d& I() const { return fI; }
    
    // pseudo inertia matrix about the parent link
    const Eigen::Matrix4d& J() const { return fJ; }
    
  private:
    void addCJoint(Joint* cJoint) { fCJoints.push_back(cJoint); }
    
    Joint* fPJoint;
    std::list<Joint*> fCJoints;
    Eigen::Vector3d fPos;
    std::string fName;
    
    double fm;
    Eigen::Matrix3d fI;
    Eigen::Matrix4d fJ;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Matrix4d S_x(double phi);
Eigen::Matrix4d S_y(double phi);
Eigen::Matrix4d S_z(double phi);
Eigen::Matrix4d dS_x_dp(double phi);
Eigen::Matrix4d dS_y_dp(double phi);
Eigen::Matrix4d dS_z_dp(double phi);
Eigen::Matrix4d d2S_x_dpp(double phi);
Eigen::Matrix4d d2S_y_dpp(double phi);
Eigen::Matrix4d d2S_z_dpp(double phi);

class Joint {
  friend class RigidBody;
  friend class KinTree;
  public:
    Joint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name);
    
    RigidBody* up()   const { return fPRB; }
    RigidBody* down() const { return fCRB; }
    
    virtual unsigned int npar() const { return 0; }
    //virtual GiNaC::matrix R(const std::vector<GiNaC::ex>& params) const;
    //virtual GiNaC::matrix dR_di(const std::vector<GiNaC::ex>& params, unsigned int i) const = 0;
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const = 0;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const = 0;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const = 0;
    
    const Eigen::Vector3d& pos() const { return fPos; }
    
    unsigned int pid(unsigned int par) const {
        if(par >= npar())
            throw std::runtime_error("ParameterID out of range");
        return par + fParamBase;
    }
    
  private:
    void setChild(RigidBody* cRB) { fCRB = cRB; }
    void setParamBase(unsigned int pbase) { fParamBase = pbase; }
    
    RigidBody* fPRB, *fCRB;
    Eigen::Vector3d fPos;
    std::string fName;
    unsigned int fParamBase;
};

class HingeJoint: public Joint {
  public:
    HingeJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 1; }
    //virtual GiNaC::matrix R(const std::vector<GiNaC::ex>& params) const;
    //virtual GiNaC::matrix dR_di(const std::vector<GiNaC::ex>& params, unsigned int i) const;
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class UniversalJoint: public Joint {
  public:
    UniversalJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 2; }
    //virtual GiNaC::matrix R(const std::vector<GiNaC::ex>& params) const;
    //virtual GiNaC::matrix dR_di(const std::vector<GiNaC::ex>& params, unsigned int i) const;
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class BallJoint: public Joint {
  public:
    BallJoint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : Joint(pRB, pos, name) {}
    virtual unsigned int npar() const { return 3; }
    //virtual GiNaC::matrix R(const std::vector<GiNaC::ex>& params) const;
    //virtual GiNaC::matrix dR_di(const std::vector<GiNaC::ex>& params, unsigned int i) const;
    
    virtual Eigen::Matrix4d S(const Eigen::VectorXd& par) const;
    virtual Eigen::Matrix4d dS_di(const Eigen::VectorXd& par, unsigned int i) const;
    virtual Eigen::Matrix4d d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const;
};

class KinTree
{
  public:
    KinTree(RigidBody* root)
        : fRBRoot(root) {}
    
    RigidBody* root() const { return fRBRoot; }
    void traverse();
    void traverse(RigidBody* rb);
    
    void AssignParamIDs();
    unsigned int AssignParamIDs(RigidBody* rb, unsigned int npar);
    
    void setG(const Eigen::Vector3d& g) { fg = g; }
    
    const Eigen::MatrixXd& M() const { return fM; }
    const Eigen::VectorXd& C() const { return fC; }
    const Eigen::VectorXd& dVdq() const { return fdVdq; }
    
    const Eigen::Vector3d& g() const { return fg; }
    
    void calcDyn(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot);
    
    unsigned int npar() const { return fNParams; }
  private:
    RigidBody* fRBRoot;
    unsigned int fNParams;
    
    Eigen::MatrixXd fM;
    Eigen::VectorXd fC;
    Eigen::VectorXd fdVdq;
    
    Eigen::Vector3d fg;
};

#endif
