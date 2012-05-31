#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "RawODESolver.h"
#include <GiJIT.h>
#include "Lagrange.h"
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"
#include <gsl/gsl_errno.h>

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
    GiNaC::matrix C(const Vector& q, const Vector& qdot) const;
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

template <typename Derived>
GiNaC::matrix fromEigen(const Eigen::MatrixBase<Derived>& e)
{
    GiNaC::matrix m(e.rows(), e.cols());
    for(unsigned int i=0; i<e.rows(); i++) {
        for(unsigned int j=0; j<e.cols(); j++) {
            m(i,j) = e(i,j);
        }
    }
    
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

/* GiNaC::matrix JointSys::M(const Vector& q) const
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
} */

GiNaC::matrix JointSys::M(const Vector& q) const
{
    using namespace GiNaC;
    unsigned int n = 0;
    
    std::vector<ex> q_ex(q.begin(), q.end());
    
    std::vector<symbol> q_dot;
    q_dot.reserve(ndim());
    for(int i=0; i<ndim(); i++)
        q_dot.push_back(symbol());
    std::vector<ex> q_dot_ex(q_dot.begin(), q_dot.end());
    
    Joint* joint = kt->root()->cJoints().front();
    RigidBody* body = joint->down();
    
    // ex E_kin = 0;
    std::vector<ex> dTk_dq(ndim());
    
    matrix Id44(4, 4);
    Id44 = 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    
    ex Tk = Id44;
    
    matrix Q(ndim(), ndim());
    
    while(1)
    {
        for(unsigned int i=0; i<joint->npar(); i++) {
            // std::cout << T.diff(q.at(i+n)).evalm() << std::endl;
            dTk_dq[i+n] = (Tk * joint->dR_di(q_ex, i)).evalm();
        }
        
        ex Sk = joint->R(q_ex);
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = (dTk_dq[i] * Sk).evalm();
        }
        
        Tk = Tk * Sk;
        
        /***
        for(unsigned int i=0; i<joint->npar(); i++) {
            dTk_dq[i+n] = Tk * joint->dS_di(par, i);
        }
        
        Eigen::Matrix4d Sk = joint->S(par);
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = dTk_dq[i] * Sk;
        } 
        
        Tk = Tk * Sk; */
        
        n += joint->npar();
        
        matrix J = fromEigen(body->J());
        
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<n; j++) {
                //ex Ti = ex_to<matrix>(Tk).evalm().diff(q.at(i));
                //ex Tj = ex_to<matrix>(Tk).evalm().diff(q.at(j));
                ex Ti = ex_to<matrix>(dTk_dq[i]).evalm();
                ex Tj = ex_to<matrix>(dTk_dq[j]).evalm();
                ex TjT = ex_to<matrix>(Tj).transpose();
                
                ex Ekin = (ex_to<matrix>((Ti * J * TjT).evalm())).trace();
                
                Q(i,j) += Ekin;
            }
        }
        
        if(body->cJoints().empty())
            break;
        
        Joint* nextJoint = body->cJoints().front();
        
        Eigen::Vector3d r0_anchor;
        r0_anchor = nextJoint->pos() - joint->pos();
        
        matrix St(4,4);
        St =  1, 0, 0, r0_anchor(0),
              0, 1, 0, r0_anchor(1),
              0, 0, 1, r0_anchor(2),
              0, 0, 0, 1;
        
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = (dTk_dq[i] * St).evalm();
        }
        Tk = (Tk * St).evalm();
        
        joint = nextJoint;
        body = joint->down();
    }
    
    return Q;
}

GiNaC::matrix JointSys::C(const Vector& q, const Vector& qdot) const
{
    using namespace GiNaC;
    unsigned int n = 0;
    
    std::vector<ex> q_ex(q.begin(), q.end());
    
    std::vector<ex> q_dot_ex(qdot.begin(), qdot.end());
    
    Joint* joint = kt->root()->cJoints().front();
    RigidBody* body = joint->down();
    
    // ex E_kin = 0;
    std::vector<ex> dTk_dq(ndim());
    
    matrix Id44(4, 4);
    Id44 = 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    
    ex Tk = Id44;
    
    matrix fC(ndim(), 1);
    
    while(1)
    {
        for(unsigned int i=0; i<joint->npar(); i++) {
            // std::cout << T.diff(q.at(i+n)).evalm() << std::endl;
            dTk_dq[i+n] = (Tk * joint->dR_di(q_ex, i)).evalm();
        }
        
        ex Sk = joint->R(q_ex);
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = (dTk_dq[i] * Sk).evalm();
        }
        
        Tk = Tk * Sk;
        
        /***
        for(unsigned int i=0; i<joint->npar(); i++) {
            dTk_dq[i+n] = Tk * joint->dS_di(par, i);
        }
        
        Eigen::Matrix4d Sk = joint->S(par);
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = dTk_dq[i] * Sk;
        } 
        
        Tk = Tk * Sk; */
        
        n += joint->npar();
        
        matrix J = fromEigen(body->J());
        
        for(unsigned int i=0; i<n; i++) {
            for(unsigned int j=0; j<n; j++) {
                for(unsigned int k=0; k<n; k++) {
                    ex T1 = ex_to<matrix>(Tk).evalm().diff(q.at(j)).diff(q.at(k));
                    ex T2 = ex_to<matrix>(Tk).evalm().diff(q.at(i));
                    //ex Ti = ex_to<matrix>(dTk_dq[i]).evalm();
                    //ex Tj = ex_to<matrix>(dTk_dq[j]).evalm();
                    ex T2T = ex_to<matrix>(T2).transpose();
                    
                    ex tmp = (ex_to<matrix>((T1 * J * T2T).evalm())).trace();
                    
                    fC(i, 0) += tmp * qdot.at(j) * qdot.at(k);
                }
            }
        }
        
        if(body->cJoints().empty())
            break;
        
        Joint* nextJoint = body->cJoints().front();
        
        Eigen::Vector3d r0_anchor;
        r0_anchor = nextJoint->pos() - joint->pos();
        
        matrix St(4,4);
        St =  1, 0, 0, r0_anchor(0),
              0, 1, 0, r0_anchor(1),
              0, 0, 1, r0_anchor(2),
              0, 0, 0, 1;
        
        for(unsigned int i=0; i<n; i++) {
            dTk_dq[i] = (dTk_dq[i] * St).evalm();
        }
        Tk = (Tk * St).evalm();
        
        joint = nextJoint;
        body = joint->down();
    }
    
    return fC;
}



GiNaC::ex JointSys::V(const Vector& q) const
{
    using namespace GiNaC;
    
    std::vector<ex> q_ex(q.begin(), q.end());
    
    Joint* joint = kt->root()->cJoints().front();
    RigidBody* body = joint->down();
    
    ex V = 0;
    
    matrix T_init(4, 4);
    T_init = 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    
    ex T = T_init;
    
    while(1)
    {
        matrix r0_cog(4,1);
        Eigen::Vector3d tmp = body->pos() - joint->pos();
        r0_cog = tmp.x(), tmp.y(), tmp.z(), 1;
        
        T = ex_to<matrix>((T * joint->R(q_ex)).evalm());
        ex r_cog = (T * r0_cog).evalm();
        
        V += body->m() * ex_to<matrix>(r_cog)(2,0);
        
        if(body->cJoints().empty())
            break;
        
        Joint* nextJoint = body->cJoints().front();
        
        Eigen::Vector3d tmp2 = nextJoint->pos() - joint->pos();
        
        matrix T_t(4,4);
        T_t = 1, 0, 0, tmp2.x(),
              0, 1, 0, tmp2.y(),
              0, 0, 1, tmp2.z(),
              0, 0, 0, 1;
        T = T*T_t;
        
        joint = nextJoint;
        body = joint->down();
    }
    
    return V;
}

GiNaC::ex JointSys::u(const Vector& q, const Vector& qdot) const
{
    return 0;
}

typedef GiJIT::CodeGenV<GiJIT::Result,
                        GiJIT::Vector > M_compiler;

/* typedef GiJIT::CodeGenV<GiJIT::Result,
                        GiJIT::Vector,
                        GiJIT::Vector > M_qdotdot_compiler; */

typedef GiJIT::CodeGenV<GiJIT::Result,
                        GiJIT::Vector,
                        GiJIT::Vector > C_compiler;

typedef GiJIT::CodeGenV<GiJIT::Result,
                        GiJIT::Vector > dVdq_compiler;

struct JointSysData {
    //M_compiler::func_t calc_M;
    // M_qdotdot_compiler::func_t calc_M_qdotdot;
    //C_compiler::func_t calc_C;
    //dVdq_compiler::func_t calc_dVdq;
    
    KinTree* kt;
    unsigned int ndim;
};

int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    struct JointSysData* data = (struct JointSysData*) _p;
    Eigen::Map<const Eigen::VectorXd> q(x_raw, data->ndim);
    Eigen::Map<const Eigen::VectorXd> qdot(x_raw+data->ndim, data->ndim);
    
    // Eigen::MatrixXd M(data->ndim, data->ndim);
    //data->calc_M(M.data(), q.data());   // calculate M(q)
    data->kt->calcDyn(q, qdot);
    Eigen::MatrixXd M = data->kt->M();
    
    Eigen::VectorXd C(data->ndim);
    // data->calc_C(C.data(), q.data(), qdot.data());
    C = data->kt->C();
    
    Eigen::VectorXd dVdq(data->ndim);
    //data->calc_dVdq(dVdq.data(), q.data());
    dVdq = data->kt->dVdq();
    
    //Eigen::VectorXd M_qdotdot(data->ndim);
    //data->calc_M_qdotdot(M_qdotdot.data(), q.data(), qdot.data());   // calculate M(q) * qdotdot(q, qdot)
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (-C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(-C - dVdq);
    
    return GSL_SUCCESS;
}


int main()
{
    using namespace GiNaC;
    
    struct JointSysData fData;
    
    RigidBody* body0 = new RigidBody(0, Eigen::Vector3d(0., 0., 0.), "body0");
    BallJoint* joint1 = new BallJoint(body0, Eigen::Vector3d(0., 0., 0.), "joint1");
    RigidBody* body1 = new RigidBody(joint1, Eigen::Vector3d(0., 0., -1.), "body1");
    body1->setMass(1., 1., .5, .7, 0., 0., 0.);
    
    HingeJoint* joint2 = new HingeJoint(body1, Eigen::Vector3d(0., 0., -1.5), "joint2");
    RigidBody* body2 = new RigidBody(joint2, Eigen::Vector3d(0., 0., -2.), "body2");
    body2->setMass(.5, .1, .2, .3, 0., 0., 0.);
    
    UniversalJoint* joint3 = new UniversalJoint(body2, Eigen::Vector3d(0., 0., -2.5), "joint3");
    RigidBody* body3 = new RigidBody(joint3, Eigen::Vector3d(0., 0., -3.), "body3");
    body3->setMass(.9, .5, .4, .3, 0., 0., 0.);
    
    KinTree kt(body0);
    kt.AssignParamIDs();
    kt.setG(Eigen::Vector3d(0., 0., -1.));
    
    //JointSys sys(&kt);
    //Lagrange L(sys);
    
    const double x_ini[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, -.5,  .7, 0.0, 0.0, 0.0 };
    const double tstep = 1./16.;
    int i;
    
    // LagrODESolver solver(sys.ndim(), L.M_qdotdot(), L.M(), L.q(), L.qdot(), q_ini, qdot_ini);
    
    //fData.calc_M = M_compiler::compile(sys.M(L.q()), L.q());
    // fData.calc_M_qdotdot = M_qdotdot_compiler::compile(L.M_qdotdot(), L.q(), L.qdot());
    //fData.calc_C = C_compiler::compile(sys.C(L.q(), L.qdot()), L.q(), L.qdot());
    //fData.calc_dVdq = dVdq_compiler::compile(L.dVdQ(), L.q());
    
    fData.kt = &kt;
    fData.ndim = kt.npar();
    
    RawODESolver solver(2*kt.npar(), calc_xdot, 0, x_ini, &fData);
    
    std::cout << "#:1:phi_ana" << std::endl;
    std::cout << "#:2:theta_ana" << std::endl;
    std::cout << "#:3:psi_ana" << std::endl;
    std::cout << "#:4:alpha_ana" << std::endl;
    std::cout << "#:5:beta_ana" << std::endl;
    std::cout << "#:6:gamma_ana" << std::endl;
    
    //kt.M(x_ini);
    
    for(i=0; i<(16*10); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << solver.y()[0] << " ";
        std::cout << solver.y()[1] << " ";
        std::cout << solver.y()[2] << " ";
        std::cout << solver.y()[3] << " ";
        std::cout << solver.y()[4] << " ";
        std::cout << solver.y()[5] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
