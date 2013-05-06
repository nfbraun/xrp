#include "KinTree.h"
#include <iostream>
#include <cmath>

RigidBody::RigidBody(Joint* pJoint, const Eigen::Vector3d& pos, const std::string& name)
        : fPJoint(pJoint), fPos(pos), fName(name)
{
    if(pJoint)
        pJoint->setChild(this);
}

void RigidBody::setMass(double m, double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz)
{
    fm = m;
    fI << Ixx, Ixy, Ixz,
          Ixy, Iyy, Iyz,
          Ixz, Iyz, Izz;
    
    Eigen::Vector3d rp = pos();
    if(pJoint())
        rp -= pJoint()->pos();
    
    fJ << -fI + m*rp*rp.transpose() + fI.trace()/2.*Eigen::Matrix3d::Identity(), m*rp,
          m*rp.transpose(),                                                      m;
}

void RigidBody::setMass(double m, const Eigen::Vector3d& Idiag)
{
    setMass(m, Idiag.x(), Idiag.y(), Idiag.z(), 0., 0., 0.);
}

void RigidBody::setMassBox(double density, double sx, double sy, double sz)
{
    const double m = density * sx * sy * sz;
    setMass(m, m/12.*(sy*sy+sz*sz), m/12.*(sx*sx+sz*sz), m/12.*(sy*sy+sz*sz),
            0., 0., 0.);
}

void RigidBody::setMassCylinder(double density, double h, double r)
{
    const double m = density * h * M_PI * r * r;
    setMass(m, m/12.*(3*r*r + h*h), m/12.*(3*r*r + h*h), m/2.*r*r, 0., 0., 0.);
}

Joint::Joint(RigidBody* pRB, const Eigen::Vector3d& pos, const std::string name)
        : fPRB(pRB), fPos(pos), fName(name), fCRB(0)
{
    if(pRB)
        pRB->addCJoint(this);
    else
        std::cerr << "Creating dangling joint" << std::endl;
}

void KinTree::traverse()
{
    traverse(root());
}

void KinTree::traverse(RigidBody* rb)
{
    std::cout << rb->name() << std::endl;
    for(std::list<Joint*>::const_iterator it = rb->cJoints().begin();
        it != rb->cJoints().end(); it++) {
        if(!(*it)->down()) {
            std::cerr << "Dangling joint" << std::endl;
            continue;
        }
        traverse((*it)->down());
    }
}

void KinTree::AssignParamIDs()
{
    fNParams = AssignParamIDs(root(), 0);
}

unsigned int KinTree::AssignParamIDs(RigidBody* rb, unsigned int npar)
{
    for(std::list<Joint*>::const_iterator it = rb->cJoints().begin();
        it != rb->cJoints().end(); it++) {
        Joint* j = *it;
        if(!j->down()) {
            std::cerr << "Dangling joint" << std::endl;
            continue;
        }
        
        j->setParamBase(npar);
        npar += j->npar();
        
        npar = AssignParamIDs(j->down(), npar);
    }
    
    return npar;
}

Eigen::Matrix4d S_x(double phi)
{
    Eigen::Matrix4d S;
    S << 1,        0,         0, 0,
         0, cos(phi), -sin(phi), 0,
         0, sin(phi),  cos(phi), 0,
         0,        0,         0, 1;
    return S;
}

Eigen::Matrix4d S_y(double phi)
{
    Eigen::Matrix4d S;
    S << cos(phi),  0, sin(phi), 0,
         0,         1,        0, 0,
         -sin(phi), 0, cos(phi), 0,
         0,         0,        0, 1;
    return S;
}

Eigen::Matrix4d S_z(double phi)
{
    Eigen::Matrix4d S;
    S << cos(phi), -sin(phi), 0, 0,
         sin(phi),  cos(phi), 0, 0,
         0,         0,        1, 0,
         0,         0,        0, 1;
    return S;
}

Eigen::Matrix4d dS_x_dp(double phi)
{
    Eigen::Matrix4d S;
    S << 0,        0,          0, 0,
         0, -sin(phi), -cos(phi), 0,
         0,  cos(phi), -sin(phi), 0,
         0,        0,          0, 0;
    return S;
}

Eigen::Matrix4d dS_y_dp(double phi)
{
    Eigen::Matrix4d S;
    S << -sin(phi), 0,  cos(phi), 0,
         0,         0,         0, 0,
         -cos(phi), 0, -sin(phi), 0,
         0,         0,         0, 0;
    return S;
}

Eigen::Matrix4d dS_z_dp(double phi)
{
    Eigen::Matrix4d S;
    S << -sin(phi), -cos(phi), 0, 0,
          cos(phi), -sin(phi), 0, 0,
         0,         0,         0, 0,
         0,         0,         0, 0;
    return S;
}

Eigen::Matrix4d d2S_x_dpp(double phi)
{
    Eigen::Matrix4d S;
    S << 0,        0,          0, 0,
         0, -cos(phi),  sin(phi), 0,
         0, -sin(phi), -cos(phi), 0,
         0,        0,          0, 0;
    return S;
}

Eigen::Matrix4d d2S_y_dpp(double phi)
{
    Eigen::Matrix4d S;
    S << -cos(phi), 0, -sin(phi), 0,
         0,         0,         0, 0,
          sin(phi), 0, -cos(phi), 0,
         0,         0,         0, 0;
    return S;
}

Eigen::Matrix4d d2S_z_dpp(double phi)
{
    Eigen::Matrix4d S;
    S << -cos(phi),  sin(phi), 0, 0,
         -sin(phi), -cos(phi), 0, 0,
         0,         0,         0, 0,
         0,         0,         0, 0;
    return S;
}

Eigen::Matrix4d HingeJoint::S(const Eigen::VectorXd& par) const
{
    return S_x(par[pid(0)]);
}

Eigen::Matrix4d UniversalJoint::S(const Eigen::VectorXd& par) const
{
    return S_y(par[pid(0)]) * S_x(par[pid(1)]);
}

Eigen::Matrix4d BallJoint::S(const Eigen::VectorXd& par) const
{
    return S_z(par[pid(0)]) * S_y(par[pid(1)]) * S_x(par[pid(2)]);
}

Eigen::Matrix4d HingeJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_x_dp(par[pid(0)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d UniversalJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_y_dp(par[pid(0)]) * S_x(par[pid(1)]);
        case 1: return S_y(par[pid(0)]) * dS_x_dp(par[pid(1)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d BallJoint::dS_di(const Eigen::VectorXd& par, unsigned int i) const
{
    switch(i) {
        case 0: return dS_z_dp(par[pid(0)]) * S_y(par[pid(1)]) * S_x(par[pid(2)]);
        case 1: return S_z(par[pid(0)]) * dS_y_dp(par[pid(1)]) * S_x(par[pid(2)]);
        case 2: return S_z(par[pid(0)]) * S_y(par[pid(1)]) * dS_x_dp(par[pid(2)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

Eigen::Matrix4d HingeJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 0 || j > 0) throw std::runtime_error("ParameterID out of range");
    return d2S_x_dpp(par[pid(0)]);
}

Eigen::Matrix4d UniversalJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
{
    if(i > 1 || j > 1) throw std::runtime_error("ParameterID out of range");
    switch(2*i+j) {
        case 0:         return d2S_y_dpp(par[pid(0)]) * S_x(par[pid(1)]);
        case 1: case 2: return dS_y_dp(par[pid(0)]) * dS_x_dp(par[pid(1)]);
        case 3:         return S_y(par[pid(0)]) * d2S_x_dpp(par[pid(1)]);
    }
}

Eigen::Matrix4d BallJoint::d2S_dij(const Eigen::VectorXd& par, unsigned int i, unsigned int j) const
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

Eigen::Matrix4d S_translate(const Eigen::Vector3d& t)
{
    Eigen::Matrix4d S;
    S << 1, 0, 0, t.x(),
         0, 1, 0, t.y(),
         0, 0, 1, t.z(),
         0, 0, 0, 1;
    return S;
}

unsigned int idx(unsigned int i, unsigned int j)
{
    return (i*(i+1))/2 + j;
}

void KinTree::calcDyn(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
{
    unsigned int n = 0;
    
    /* the mass matrix, defined via
M_{ij} = \sum_{bodies p} tr( \frac{d T_p}{d q_i} J_p (\frac{d T_p}{d q_j})^T )
    */
    fM.setZero(npar(), npar());
    fC.setZero(npar());
    fdVdq.setZero(npar());
    
    Eigen::Matrix4d Tp = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d dTp_dq[npar()];
    const int d2Tp_dqq_size = (npar() * (npar() + 1)) / 2;
    Eigen::Matrix4d d2Tp_dqq[d2Tp_dqq_size];
    
    Joint* joint = root()->cJoints().front();
    RigidBody* body = joint->down();
    
    while(1)
    {
        // Move past the joint
        for(unsigned int i=0; i<joint->npar(); i++) {
            for(unsigned int j=0; j<n; j++) {
                d2Tp_dqq[idx(i+n, j)] = dTp_dq[j] * joint->dS_di(q, i);
            }
        }
        
        for(unsigned int i=0; i<joint->npar(); i++) {
            for(unsigned int j=0; j<=i; j++) {
                d2Tp_dqq[idx(i+n, j+n)] = Tp * joint->d2S_dij(q, i, j);
            }
        }
        
        for(unsigned int i=0; i<joint->npar(); i++) {
            dTp_dq[i+n] = Tp * joint->dS_di(q, i);
        }
        
        Eigen::Matrix4d Sp = joint->S(q);
        
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<=i; j++) {
                d2Tp_dqq[idx(i, j)] = d2Tp_dqq[idx(i, j)] * Sp;
            }
        }
        
        for(unsigned int i=0; i<n; i++) {
            dTp_dq[i] = dTp_dq[i] * Sp;
        } 
        
        Tp = Tp * Sp;
        
        n += joint->npar();
        
        // Calculate this bodies contribution to mass matrix
        Eigen::Matrix4d Jp = body->J();
        
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<n; j++) {
                fM(i,j) += (dTp_dq[i] * Jp * dTp_dq[j].transpose()).trace();
            }
        }
        
        // Calculate this bodies contribution to the Coriolis vector
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<n; j++) {
                fC(i) += (d2Tp_dqq[idx(j, j)] * Jp * dTp_dq[i].transpose()).trace()
                        * qdot(j) * qdot(j);
                for(unsigned int k=0; k<j; k++) {
                    fC(i) += 2 * (d2Tp_dqq[idx(j, k)] * Jp * dTp_dq[i].transpose()).trace()
                        * qdot(j) * qdot(k);
                }
            }
        }
        
        // Calculate this bodies contribution to the gravity term
        for(unsigned int i=0; i<n; i++) {
            Eigen::Vector4d rp;
            rp << (body->pos() - joint->pos()), 1.;
            fdVdq(i) += -body->m() * fg.dot((dTp_dq[i] * rp).head<3>());
        }
        
        
        if(body->cJoints().empty())
            break;
        
        // Move past the body
        Joint* nextJoint = body->cJoints().front();
        
        Eigen::Matrix4d St = S_translate(nextJoint->pos() - joint->pos());
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<=i; j++) {
                d2Tp_dqq[idx(i, j)] = d2Tp_dqq[idx(i, j)] * St;
            }
        }
        for(unsigned int i=0; i<n; i++) {
            dTp_dq[i] = dTp_dq[i] * St;
        }
        Tp = Tp * St;
        
        joint = nextJoint;
        body = joint->down();
    }
}

/* *** */

/* GiNaC::matrix R_x(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R = 1,        0,         0, 0,
        0, cos(phi), -sin(phi), 0,
        0, sin(phi),  cos(phi), 0,
        0,        0,         0, 1;
    return R;
}

GiNaC::matrix R_y(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R = cos(phi),  0, sin(phi), 0,
        0,         1,        0, 0,
        -sin(phi), 0, cos(phi), 0,
        0,         0,        0, 1;
    return R;
}

GiNaC::matrix R_z(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R = cos(phi), -sin(phi), 0, 0,
        sin(phi),  cos(phi), 0, 0,
        0,         0,        1, 0,
        0,         0,        0, 1;
    return R;
}

GiNaC::matrix dR_x_dp(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R =  0,        0,          0, 0,
         0, -sin(phi), -cos(phi), 0,
         0,  cos(phi), -sin(phi), 0,
         0,        0,          0, 0;
    return R;
}

GiNaC::matrix dR_y_dp(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R = -sin(phi), 0,  cos(phi), 0,
         0,         0,         0, 0,
         -cos(phi), 0, -sin(phi), 0,
         0,         0,         0, 0;
    return R;
}

GiNaC::matrix dR_z_dp(GiNaC::ex phi)
{
    GiNaC::matrix R(4,4);
    R =  -sin(phi), -cos(phi), 0, 0,
          cos(phi), -sin(phi), 0, 0,
         0,         0,         0, 0,
         0,         0,         0, 0;
    return R;
}


GiNaC::matrix Joint::R(const std::vector<GiNaC::ex>& params) const
{
    GiNaC::matrix mat(4, 4);
    mat = 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    return mat;
}

GiNaC::matrix HingeJoint::R(const std::vector<GiNaC::ex>& params) const
{
    return R_x(params.at(pid(0)));
}

GiNaC::matrix UniversalJoint::R(const std::vector<GiNaC::ex>& params) const
{
    return GiNaC::ex_to<GiNaC::matrix>((R_y(params.at(pid(0))) * R_x(params.at(pid(1)))).evalm());
}

GiNaC::matrix BallJoint::R(const std::vector<GiNaC::ex>& params) const
{
    return GiNaC::ex_to<GiNaC::matrix>((R_z(params.at(pid(0))) * R_y(params.at(pid(1))) * R_x(params.at(pid(2)))).evalm());
}

GiNaC::matrix HingeJoint::dR_di(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    GiNaC::ex R;
    switch(i) {
        case 0: R = dR_x_dp(par[pid(0)]); break;
        default: throw std::runtime_error("ParameterID out of range");
    }
    return GiNaC::ex_to<GiNaC::matrix>(R.evalm());
}

GiNaC::matrix UniversalJoint::dR_di(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    GiNaC::ex R;
    switch(i) {
        case 0: R = dR_y_dp(par[pid(0)]) * R_x(par[pid(1)]); break;
        case 1: R = R_y(par[pid(0)]) * dR_x_dp(par[pid(1)]); break;
        default: throw std::runtime_error("ParameterID out of range");
    }
    return GiNaC::ex_to<GiNaC::matrix>(R.evalm());
}

GiNaC::matrix BallJoint::dR_di(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    GiNaC::ex R;
    switch(i) {
        case 0: R = dR_z_dp(par[pid(0)]) * R_y(par[pid(1)]) * R_x(par[pid(2)]); break;
        case 1: R = R_z(par[pid(0)]) * dR_y_dp(par[pid(1)]) * R_x(par[pid(2)]); break;
        case 2: R = R_z(par[pid(0)]) * R_y(par[pid(1)]) * dR_x_dp(par[pid(2)]); break;
        default: throw std::runtime_error("ParameterID out of range");
    }
    return GiNaC::ex_to<GiNaC::matrix>(R.evalm());
} */

/* GiNaC::matrix HingeJoint::Ri(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    switch(i) {
        case 0: return R_x(par[pid(0)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

GiNaC::matrix UniversalJoint::Ri(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    switch(i) {
        case 0: return R_y(par[pid(0)]);
        case 1: return R_x(par[pid(1)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
}

GiNaC::matrix BallJoint::Ri(const std::vector<GiNaC::ex>& par, unsigned int i) const
{
    switch(i) {
        case 0: return R_z(par[pid(0)]);
        case 1: return R_y(par[pid(1)]);
        case 2: return R_x(par[pid(2)]);
        default: throw std::runtime_error("ParameterID out of range");
    }
} */
//Vector KinTree::getWorldPos(RigidBody* rb)
//{
    /* Vector wpos;
    wpos += pJoint()->pos() - pos();
    
    wpos += pJoint()->up()->R() * (pJoint->up()->pos() - pJoint->pos()) */
//}
