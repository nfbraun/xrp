#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "LagrODESolver.h"
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
    
    virtual int ndim() const { return 4; }
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

GiNaC::ex simplify(const GiNaC::ex& ex)
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
    assert(ndim() == 4);
    
    symbol phi = q.at(0);
    symbol theta = q.at(1);
    symbol psi = q.at(2);
    symbol alpha = q.at(3);
    
    matrix r0(3,1);
    r0 = 0, 0, -1;
    
    symbol phi_dot("phi_dot"), theta_dot("theta_dot"), psi_dot("psi_dot");
    symbol alpha_dot("alpha_dot");
    matrix q_dot(ndim(),1);
    q_dot = phi_dot, theta_dot, psi_dot, alpha_dot;
    
    const double I00 = 1., I11 = .5, I22 = .7;
    
    matrix R = ex_to<matrix>((R_z(psi) * R_y(theta) * R_x(phi)).evalm());
    ex r = (R * r0).evalm();
    
    matrix r_dot = ex_to<matrix>((
            r.diff(phi) * phi_dot + r.diff(theta) * theta_dot + r.diff(psi) * psi_dot
        ).evalm());
    
    matrix R_dot = ex_to<matrix>((
            R.diff(phi) * phi_dot + R.diff(theta) * theta_dot + R.diff(psi) * psi_dot
        ).evalm());
    
    matrix Omega = ex_to<matrix>((R.transpose() * R_dot).evalm());
    matrix omega(3,1);
    omega = simplify(-Omega(1,2)), simplify(Omega(0,2)), -simplify(Omega(0,1));
    
    matrix r0_anchor(3,1);
    r0_anchor = 0, 0, -1.5;
    
    matrix r0_cog(3, 1);
    r0_cog = 0, 0, -0.5;
    
    matrix R2 = ex_to<matrix>((R * R_x(alpha)).evalm());
    matrix r2 = ex_to<matrix>((R * r0_anchor + R2 * r0_cog).evalm());
    
    matrix r2_dot = ex_to<matrix>((
            r2.diff(phi) * phi_dot + r2.diff(theta) * theta_dot + r2.diff(psi) * psi_dot
             + r2.diff(alpha) * alpha_dot
        ).evalm());
    
    matrix R2_dot = ex_to<matrix>((
            R2.diff(phi) * phi_dot + R2.diff(theta) * theta_dot + R2.diff(psi) * psi_dot
             + R2.diff(alpha) * alpha_dot
        ).evalm());
    
    matrix Omegap = ex_to<matrix>((R2.transpose() * R2_dot).evalm());
    matrix omegap(3,1);
    omegap = simplify(-Omegap(1,2)), simplify(Omegap(0,2)), -simplify(Omegap(0,1));
    
    ex T_rot = (
        omega(0,0)*I00*omega(0,0) + omega(1,0)*I11*omega(1,0) + omega(2,0)*I22*omega(2,0)
        + omegap(0,0)*0.1*omegap(0,0) + omegap(1,0)*0.2*omegap(1,0) + omegap(2,0)*0.3*omegap(2,0)
        ).expand();
    ex T = T_rot + simplify(dot(r_dot, r_dot)) + 0.5 * simplify(dot(r2_dot, r2_dot));
    
    std::cerr << make_quadratic(T, q_dot) << std::endl;
    
    return make_quadratic(T, q_dot);
}

GiNaC::ex JointSys::V(const Vector& q) const
{
    using namespace GiNaC;
    
    symbol phi = q.at(0);
    symbol theta = q.at(1);
    symbol psi = q.at(2);
    
    symbol alpha = q.at(3);
    
    matrix r0(3,1);
    r0 = 0, 0, -1;
    matrix R = ex_to<matrix>((R_z(psi) * R_y(theta) * R_x(phi)).evalm());
    matrix r = ex_to<matrix>((R * r0).evalm());
    
    matrix r0_anchor(3,1);
    r0_anchor = 0, 0, -1.5;
    
    matrix r0_cog(3, 1);
    r0_cog = 0, 0, -0.5;
    
    matrix R2 = R_x(alpha);
    matrix r2 = ex_to<matrix>((R * r0_anchor + R * R2 * r0_cog).evalm());
    
    return r(2,0) + 0.5*r2(2,0);
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
    
    const double q_ini[] = { 0.0, 0.0, 0.0, 0.0 };
    const double qdot_ini[] = { .7, -.5, 0., 0. };
    const double tstep = 1./16.;
    int i;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    LagrODESolver solver(sys.ndim(), L.M_qdotdot(), L.M(), L.q(), L.qdot(), q_ini, qdot_ini);
    
    std::cout << "#:1:phi_ana" << std::endl;
    std::cout << "#:2:theta_ana" << std::endl;
    std::cout << "#:3:psi_ana" << std::endl;
    std::cout << "#:4:alpha_ana" << std::endl;
    
    for(i=0; i<(16*10); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << solver.q()[0] << " ";
        std::cout << solver.q()[1] << " ";
        std::cout << solver.q()[2] << " ";
        std::cout << solver.q()[3] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
