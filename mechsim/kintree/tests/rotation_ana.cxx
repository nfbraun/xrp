#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "ODESolver.h"
#include <GiJIT.h>
#include "Lagrange.h"
#include <cassert>

class JointSys: public System
{
  public:
    JointSys() : System() {}
    
    GiNaC::ex dot(GiNaC::ex u, GiNaC::ex v) const;
    GiNaC::matrix R_x(GiNaC::ex phi) const;
    GiNaC::matrix R_y(GiNaC::ex phi) const;
    GiNaC::matrix R_z(GiNaC::ex phi) const;
    
    GiNaC::ex simplify(const GiNaC::ex& ex) const;
    
    virtual int ndim() const { return 3; }
    virtual GiNaC::matrix M_quat(const Vector& q) const;
    virtual GiNaC::matrix M(const Vector& q) const;
    virtual GiNaC::ex V(const Vector& q) const;
    virtual GiNaC::ex u(const Vector& q, const Vector& qdot) const;
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

GiNaC::matrix JointSys::M_quat(const Vector& q) const
{
    using namespace GiNaC;
    assert(ndim() == 3);
    
    symbol b = q.at(0);
    symbol c = q.at(1);
    symbol d = q.at(2);
    
    ex a = sqrt(1 - b*b - c*c - d*d);
    
    matrix r0(3,1);
    r0 = 0, 0, -1;
    
    const int I00 = 3, I11 = 1, I22 = 2;
    
    matrix R(3, 3);
    R = a*a + b*b - c*c - d*d,        2*b*c - 2*a*d,         2*b*d + 2*a*c,
                2*b*c + 2*a*d, a*a - b*b + c*c -d*d,         2*c*d - 2*a*b,
                2*b*d - 2*a*c,        2*c*d + 2*a*b, a*a - b*b - c*c + d*d;
    ex r = (R * r0).evalm();
    
    symbol b_dot("b_dot"), c_dot("c_dot"), d_dot("d_dot");
    ex a_dot = -(b*b_dot + c*c_dot + d*d_dot)/a;
    
    matrix omega(3,1);
    omega = a*b_dot - b*a_dot - c*d_dot + d*c_dot,
            a*c_dot + b*d_dot - c*a_dot - d*b_dot,
            a*d_dot - b*c_dot + c*b_dot - d*a_dot;
    ex T_rot = (omega(0,0)*I00*omega(0,0) + omega(1,0)*I11*omega(1,0) + omega(2,0)*I22*omega(2,0)).expand();
    
    GiNaC::matrix mat(ndim(), ndim());
    mat = T_rot.coeff(b_dot, 2),                   T_rot.coeff(b_dot, 1).coeff(c_dot, 1)/2, T_rot.coeff(b_dot, 1).coeff(d_dot, 1)/2,
          T_rot.coeff(c_dot, 1).coeff(b_dot, 1)/2, T_rot.coeff(c_dot, 2),                   T_rot.coeff(c_dot, 1).coeff(d_dot, 1)/2,
          T_rot.coeff(d_dot, 1).coeff(b_dot, 1)/2, T_rot.coeff(d_dot, 1).coeff(c_dot, 1)/2, T_rot.coeff(d_dot, 2);
    /* mat = T_rot.coeff(b_dot, 2), T_rot.coeff(b_dot,1).coeff(c_dot,1)/2, T_rot.coeff(b_dot, 1).coeff(d_dot, 1)/2,
          T_rot.coeff(b_dot,1).coeff(c_dot,1)/2, T_rot.coeff(c_dot, 2), 0,
          T_rot.coeff(b_dot, 1).coeff(d_dot, 1)/2, 0, T_rot.coeff(d_dot, 2); */
          
    std::cout << mat << std::endl;
    
    return mat;
}

GiNaC::ex JointSys::simplify(const GiNaC::ex& ex) const
{
    using namespace GiNaC;
    
    GiNaC::ex tmp = ex.expand();
    tmp = tmp.subs(cos(wild()) == sqrt(1 - pow(sin(wild()), 2)));
    tmp = tmp.expand();
    tmp = tmp.subs(sqrt(1 - pow(sin(wild()), 2)) == cos(wild()));
    
    return tmp;
}

GiNaC::matrix make_quadratic(const GiNaC::ex& y, const GiNaC::matrix& x)
// returns a matrix Q such that y = x^T Q x
// TODO: this is not error-proof (does not check degree of y, etc.)
{
    assert(x.cols() == 1);
    
    GiNaC::matrix Q(x.rows(), x.rows());
    
    GiNaC::lst x_lst;
    for(unsigned int i=0; i<x.rows(); i++)
        x_lst.append(x(i,0));
    
    assert(y.is_polynomial(x_lst));
    
    // extract off-diagonal elements
    for(unsigned int i=0; i<x.rows(); i++) {
        for(unsigned int j=i+1; j<x.rows(); j++) {
            Q(i,j) = y.coeff(x(i,0), 1).coeff(x(j,0), 1)/2;
            Q(j,i) = Q(i,j);
        }
    }
    
    // extract diagonal elements
    for(unsigned int i=0; i<x.rows(); i++) {
        Q(i,i) = y.coeff(x(i,0), 2);
    }
    
    return Q;
}

GiNaC::matrix JointSys::M(const Vector& q) const
{
    using namespace GiNaC;
    assert(ndim() == 3);
    
    symbol phi = q.at(0);
    symbol theta = q.at(1);
    symbol psi = q.at(2);
    
    symbol phi_dot("phi_dot"), theta_dot("theta_dot"), psi_dot("psi_dot");
    matrix q_dot(3,1);
    q_dot = phi_dot, theta_dot, psi_dot;
    
    //matrix R = ex_to<matrix>((R_z(phi) * R_x(theta) * R_z(psi)).evalm());
    matrix R = ex_to<matrix>((R_z(phi) * R_y(theta) * R_x(psi)).evalm());
    
    //std::cout << R << std::endl;
    
    matrix R_dot = ex_to<matrix>((
            R.diff(phi) * phi_dot + R.diff(theta) * theta_dot + R.diff(psi) * psi_dot
        ).evalm());
    
    matrix Omega = ex_to<matrix>((R_dot * R.transpose()).evalm());
    matrix omega(3,1);
    omega = simplify(-Omega(1,2)), simplify(Omega(0,2)), -simplify(Omega(0,1));
    
    const double I00 = 3, I11 = 1, I22 = 2;
    
    ex T_rot = (omega(0,0)*I00*omega(0,0) + omega(1,0)*I11*omega(1,0) + omega(2,0)*I22*omega(2,0)).expand();
    
    return make_quadratic(T_rot, q_dot);
}

GiNaC::ex JointSys::V(const Vector& q) const
{
    return 0;
}

GiNaC::ex JointSys::u(const Vector& q, const Vector& qdot) const
{
    return 0;
}

int main()
{
    using namespace GiNaC;
    
    JointSys sys;
    Lagrange L(sys);
    
    // std::vector<symbol> q = L.q(), qdot = L.qdot();
    
    GiNaC::ex qddot = L.qdotdot();
    
    const double q_ini[] = { 0.0, 0.0, 0.0 };
    const double qdot_ini[] = { -0.7, -0.5, -1. };
    const double tstep = 1./16.;
    int i;
    
    //std::cout << qddot << std::endl;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    AutODE2Solver solver(3, qddot, L.q(), L.qdot(), q_ini, qdot_ini);
    
    std::cout << "#:1:phi_ana" << std::endl;
    std::cout << "#:2:theta_ana" << std::endl;
    std::cout << "#:3:psi_ana" << std::endl;
    
    for(i=0; i<(16*10); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        
        const double phi = solver.q()[0];
        const double theta = solver.q()[1];
        const double psi = solver.q()[2];
        
        const double phi_dot = solver.qdot()[0];
        const double theta_dot = solver.qdot()[1];
        const double psi_dot = solver.qdot()[2];
        
        std::cout << phi << " " << theta << " " << psi << " ";
        
        //std::cout << sqrt(1 - b*b - c*c - d*d) << " ";
        //std::cout << b << " " << c << " " << d << " ";
        //std::cout << solver.qdot()[0] << " ";
        //std::cout << solver.qdot()[1] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
