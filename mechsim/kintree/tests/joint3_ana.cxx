#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "LagrODESolver.h"
#include <GiJIT.h>
#include "Lagrange.h"
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"

class JointSys: public System
{
  public:
    JointSys(KinTree* kt_) : System(), kt(kt_) {}
    
    GiNaC::ex dot(GiNaC::ex u, GiNaC::ex v) const;
    GiNaC::matrix R_x(GiNaC::ex phi) const;
    GiNaC::matrix R_y(GiNaC::ex phi) const;
    GiNaC::matrix R_z(GiNaC::ex phi) const;
    
    virtual int ndim() const { return kt->npar(); }
    virtual GiNaC::matrix M(const Vector& q) const;
    virtual GiNaC::ex V(const Vector& q) const;
    virtual GiNaC::ex u(const Vector& q, const Vector& qdot) const;
    
    KinTree* kt;
};

GiNaC::ex JointSys::dot(GiNaC::ex u, GiNaC::ex v) const
{
    using namespace GiNaC;
    
    assert(is_a<matrix>(u) && ex_to<matrix>(u).cols() == 1);
    assert(is_a<matrix>(v) && ex_to<matrix>(v).cols() == 1);
    
    return ex_to<matrix>((ex_to<matrix>(u).transpose()*v).evalm())(0,0);
}

GiNaC::matrix JointSys::R_x(GiNaC::ex phi) const
{
    GiNaC::matrix R(3,3);
    R = 1,        0,         0,
        0, cos(phi), -sin(phi),
        0, sin(phi),  cos(phi);
    return R;
}

GiNaC::matrix JointSys::R_y(GiNaC::ex phi) const
{
    GiNaC::matrix R(3,3);
    R = cos(phi),  0, sin(phi),
        0,         1,        0,
        -sin(phi), 0, cos(phi);
    return R;
}

GiNaC::matrix JointSys::R_z(GiNaC::ex phi) const
{
    GiNaC::matrix R(3,3);
    R = cos(phi), -sin(phi), 0,
        sin(phi),  cos(phi), 0,
        0,         0,        1;
    return R;
}

GiNaC::ex simplify(const GiNaC::ex& ex)
{
    using namespace GiNaC;
    
    GiNaC::ex tmp = ex.expand();
    tmp = tmp.subs(cos(wild()) == sqrt(1 - pow(sin(wild()), 2)));
    tmp = tmp.expand();
    tmp = tmp.subs(sqrt(1 - pow(sin(wild()), 2)) == cos(wild()));
    
    return tmp;
}

GiNaC::matrix make_quadratic(const GiNaC::ex& y, const std::vector<GiNaC::symbol> x)
// returns a matrix Q such that y = x^T Q x
{
    using namespace GiNaC;
    
    matrix Q(x.size(), x.size());
    exmap x_map;
    for(unsigned int i=0; i<x.size(); i++) {
        x_map[x.at(i)] = 0;
    }
    
    if(y.subs(x_map, subs_options::no_pattern) != 0)
        throw std::runtime_error("make_quadratic: y contains constant terms");
    
    for(unsigned int i=0; i<x.size(); i++) {
        symbol x_i = x.at(i);
        if(y.diff(x_i).subs(x_map, subs_options::no_pattern) != 0)
            throw std::runtime_error("make_quadratic: y contains linear terms");
    }
    
    for(unsigned int i=0; i<x.size(); i++) {
        symbol x_i = x.at(i);
        ex dy_dxi = y.diff(x_i)/2;
        for(unsigned int j=0; j<x.size(); j++) {
            symbol x_j = x.at(j);
            Q(i,j) = dy_dxi.diff(x_j);
            
            if(Q(i,j).subs(x_map, subs_options::no_pattern) != Q(i,j))
                throw std::runtime_error("make_quadratic: y contains terms of higher than quadratic order");
        }
    }
    
    return Q;
}

GiNaC::matrix fromEigen(const Eigen::Vector3d& v)
{
    GiNaC::matrix m(3, 1);
    m = v.x(), v.y(), v.z();
    
    return m;
}

GiNaC::ex dt(GiNaC::ex f, const std::vector<GiNaC::symbol>& q, const std::vector<GiNaC::ex>& q_dot)
{
    GiNaC::ex ret;
    
    for(unsigned int i=0; i<q.size(); i++) {
        ret += f.diff(q.at(i)) * q_dot.at(i);
    }
    
    return ret;
}

GiNaC::matrix JointSys::M(const Vector& q) const
{
    using namespace GiNaC;
    
    std::vector<ex> q_ex(q.begin(), q.end());
    
    std::vector<symbol> q_dot;
    q_dot.reserve(ndim());
    for(int i=0; i<ndim(); i++)
        q_dot.push_back(symbol());
    std::vector<ex> q_dot_ex(q_dot.begin(), q_dot.end());
    
    Joint* joint = kt->root()->cJoints().front();
    RigidBody* body = joint->down();
    
    matrix r(3,1);
    r = 0,0,0;
    
    ex T = 0;
    
    matrix R(3, 3);
    R = 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    
    while(1)
    {
        matrix r0_cog(3,1);
        r0_cog = fromEigen(body->pos() - joint->pos());
        
        R = ex_to<matrix>((R * joint->R(q_ex)).evalm());
        ex r_cog = r + (R * r0_cog).evalm();
        
        matrix r_dot = ex_to<matrix>(dt(r_cog, q, q_dot_ex).evalm());
        
        matrix R_dot = ex_to<matrix>(dt(R, q, q_dot_ex).evalm());
        
        matrix Omega = ex_to<matrix>((R.transpose() * R_dot).evalm());
        matrix omega(3,1);
        omega = simplify(-Omega(1,2)), simplify(Omega(0,2)), -simplify(Omega(0,1));
        
        T += body->m() * simplify(dot(r_dot, r_dot));
        
        T +=  omega(0,0)*body->Ixx()*omega(0,0)
            + omega(1,0)*body->Iyy()*omega(1,0)
            + omega(2,0)*body->Izz()*omega(2,0);
        
        if(body->cJoints().empty())
            break;
        
        Joint* nextJoint = body->cJoints().front();
        
        matrix r0_anchor(3,1);
        r0_anchor = fromEigen(nextJoint->pos() - joint->pos());
        
        r = ex_to<matrix>((r + R * r0_anchor).evalm());
        
        joint = nextJoint;
        body = joint->down();
    }
    
    return make_quadratic(T, q_dot);
}

GiNaC::ex JointSys::V(const Vector& q) const
{
    using namespace GiNaC;
    
    std::vector<ex> q_ex(q.begin(), q.end());
    
    Joint* joint = kt->root()->cJoints().front();
    RigidBody* body = joint->down();
    
    matrix r(3,1);
    r = 0,0,0;
    
    ex V = 0;
    
    matrix R(3, 3);
    R = 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    
    while(1)
    {
        matrix r0_cog(3,1);
        r0_cog = fromEigen(body->pos() - joint->pos());
        
        R = ex_to<matrix>((R * joint->R(q_ex)).evalm());
        ex r_cog = (r + R * r0_cog).evalm();
        
        V += body->m() * ex_to<matrix>(r_cog)(2,0);
        
        if(body->cJoints().empty())
            break;
        
        Joint* nextJoint = body->cJoints().front();
        
        matrix r0_anchor(3,1);
        r0_anchor = fromEigen(nextJoint->pos() - joint->pos());
        
        r = ex_to<matrix>((r + R * r0_anchor).evalm());
        
        joint = nextJoint;
        body = joint->down();
    }
    
    return V;
}

GiNaC::ex JointSys::u(const Vector& q, const Vector& qdot) const
{
    return 0;
}

int main()
{
    using namespace GiNaC;
    
    RigidBody* body0 = new RigidBody(0, Eigen::Vector3d(0., 0., 0.), "body0");
    BallJoint* joint1 = new BallJoint(body0, Eigen::Vector3d(0., 0., 0.), "joint1");
    RigidBody* body1 = new RigidBody(joint1, Eigen::Vector3d(0., 0., -1.), "body1");
    body1->setMass(1., 1., .5, .7);
    
    HingeJoint* joint2 = new HingeJoint(body1, Eigen::Vector3d(0., 0., -1.5), "joint2");
    RigidBody* body2 = new RigidBody(joint2, Eigen::Vector3d(0., 0., -2.), "body2");
    body2->setMass(.5, .1, .2, .3);
    
    UniversalJoint* joint3 = new UniversalJoint(body2, Eigen::Vector3d(0., 0., -2.5), "joint3");
    RigidBody* body3 = new RigidBody(joint3, Eigen::Vector3d(0., 0., -3.), "body3");
    body3->setMass(.9, .5, .4, .3);
    
    KinTree kt(body0);
    kt.AssignParamIDs();
    
    JointSys sys(&kt);
    Lagrange L(sys);
    
    const double q_ini[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const double qdot_ini[] = { .7, -.5, 0., 0., 0., 0. };
    const double tstep = 1./16.;
    int i;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    LagrODESolver solver(sys.ndim(), L.M_qdotdot(), L.M(), L.q(), L.qdot(), q_ini, qdot_ini);
    
    std::cout << "#:1:phi_ana" << std::endl;
    std::cout << "#:2:theta_ana" << std::endl;
    std::cout << "#:3:psi_ana" << std::endl;
    std::cout << "#:4:alpha_ana" << std::endl;
    std::cout << "#:5:beta_ana" << std::endl;
    std::cout << "#:6:gamma_ana" << std::endl;
    
    for(i=0; i<(16*10); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << solver.q()[0] << " ";
        std::cout << solver.q()[1] << " ";
        std::cout << solver.q()[2] << " ";
        std::cout << solver.q()[3] << " ";
        std::cout << solver.q()[4] << " ";
        std::cout << solver.q()[5] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
