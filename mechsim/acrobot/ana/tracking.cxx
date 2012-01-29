#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <ginac/ginac.h>
#include "ODESolver.h"
#include "Acrobot.h"
#include "LTVLQR.h"
#include <GiJIT.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_errno.h>
#include "CSVFile.h"
#include "GSLInterp.h"
#include <Eigen/Dense>
#include <stdexcept>

class NominalTrajectory {
  public:
    NominalTrajectory();
    
    GSLInterp fX[4];
    GSLInterp fU;
    double fTEnd;
};

NominalTrajectory::NominalTrajectory()
{
    CSVFile csv("acrobot.dat");
    if(csv.fail()) {
        throw std::runtime_error("Failed to open acrobot.dat");
    }
    
    std::vector<double> t;  t.reserve(csv.nval());
    std::vector<double> x[4];
    for(int i=0; i<4; i++) x[i].reserve(csv.nval());
    std::vector<double> u;  u.reserve(csv.nval());
    
    for(CSVFile::data_t::const_iterator it = csv.data().begin();
        it != csv.data().end(); it++) {
        t.push_back(it->at(0));
        x[0].push_back(it->at(1));
        x[1].push_back(it->at(2));
        x[2].push_back(it->at(3));
        x[3].push_back(it->at(4));
        u.push_back(it->at(5));
    }
    
    for(int i=0; i<4; i++)
        fX[i].init(gsl_interp_cspline, t, x[i]);
    fU.init(gsl_interp_cspline, t, u);
    
    fTEnd = csv.data().rbegin()->at(0);
}

struct GainSchedule
{
    GSLInterp fK[4];
};

typedef GiJIT::CodeGenV<GiJIT::Result,
                        GiJIT::Vector,
                        GiJIT::Vector,
                        GiJIT::Number> func_compiler;

struct Tracking
{
    Tracking(const Lagrange& l, const GiNaC::symbol& u1)
        : ltv_lqr(l, u1) {}
    
    func_compiler::func_t func;
    NominalTrajectory ntraj;
    LTVLQR ltv_lqr;
    GainSchedule gains;
};

int calc_mSdot(double t, const double S_raw[], double mSdot_raw[], void *_p)
{
    Tracking* tracking = (Tracking*) _p;

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> A;
    Eigen::Matrix<double, 4, 1> B;
    
    Eigen::Matrix<double, 4, 1> x0;
    double u0;
    if(t < tracking->ntraj.fTEnd) {
        // Stabilize to goal state
        x0 << 0, 0, 0, 0;
        u0 = 0.;
    } else {
        double tt = 2*tracking->ntraj.fTEnd - t;
        x0 << tracking->ntraj.fX[0](tt), tracking->ntraj.fX[1](tt),
              tracking->ntraj.fX[2](tt), tracking->ntraj.fX[3](tt);
        u0 = tracking->ntraj.fU(tt);
    }
    
    tracking->ltv_lqr.calcA(A.data(), x0.data(), u0);
    tracking->ltv_lqr.calcB(B.data(), x0.data(), u0);
    
    const Eigen::Matrix<double, 4, 4>& Q = tracking->ltv_lqr.Q;
    const Eigen::Matrix<double, 1, 1>& R = tracking->ltv_lqr.R;
    
    Eigen::Map<const Eigen::Matrix4d> S(S_raw);
    Eigen::Map<Eigen::Matrix4d> mSdot(mSdot_raw);
    
    mSdot = Q - S*B*R.inverse()*B.transpose()*S + S*A + A.transpose()*S;
    
    return GSL_SUCCESS;
}

int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    Tracking* tracking = (Tracking*) _p;
    
    double xbar[4];
    for(int i=0; i<4; i++)
        xbar[i] = x_raw[i] - tracking->ntraj.fX[i](t);
    
    double u = tracking->ntraj.fU(t);
    for(int i=0; i<4; i++)
        u -= tracking->gains.fK[i](t)*xbar[i];
    
    tracking->func(xdot_raw+2, x_raw, x_raw+2, u);
    xdot_raw[0] = x_raw[2];
    xdot_raw[1] = x_raw[3];
    
    return GSL_SUCCESS;
}

int calc_xbardot_linear(double t, const double xbar_raw[], double xbardot_raw[], void *_p)
{
    Tracking* tracking = (Tracking*) _p;
    
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> A;
    Eigen::Matrix<double, 4, 1> B;
    
    Eigen::Matrix<double, 4, 1> x0;
    double u0;
    x0 << tracking->ntraj.fX[0](t), tracking->ntraj.fX[1](t),
          tracking->ntraj.fX[2](t), tracking->ntraj.fX[3](t);
    u0 = tracking->ntraj.fU(t);
    
    tracking->ltv_lqr.calcA(A.data(), x0.data(), u0);
    tracking->ltv_lqr.calcB(B.data(), x0.data(), u0);
    
    Eigen::Map<const Eigen::Matrix<double, 4, 1> > xbar(xbar_raw);
    
    double ubar = 0.;
    for(int i=0; i<4; i++)
        ubar -= tracking->gains.fK[i](t)*xbar[i];
    
    Eigen::Map<Eigen::Matrix<double, 4, 1> > xbardot(xbardot_raw);
    xbardot = A*xbar + B*ubar;
    
    return GSL_SUCCESS;
}

double norm_angle(double x)
{
    while(x < 0.) x += 2.*M_PI;
    return fmod(x, 2.*M_PI);
}

void calcGainSchedule(Tracking* tracking)
{
    Eigen::Matrix<double, 4, 4> Q_f;   // final state cost
    Q_f << 0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0;
    
    RawODESolver solver(4*4, calc_mSdot, 0, Q_f.data(), tracking,
        gsl_odeiv_step_rkf45);
    
    std::vector<double> t;
    std::vector<double> k[4];
    t.resize(256);
    for(int i=0; i<4; i++) k[i].resize(256);
    
    // Solve for stationary state
    solver.EvolveFwd(tracking->ntraj.fTEnd);
    
    const double tstep = tracking->ntraj.fTEnd / 256.;
    
    for(int tidx = 0; tidx <= 255; tidx++) {
        Eigen::Map<const Eigen::Matrix<double, 4, 4> > S(solver.y());
        solver.EvolveFwd(tracking->ntraj.fTEnd + tidx * tstep);
        
        Eigen::Matrix<double, 1, 4> K;
        
        Eigen::Matrix<double, 4, 1> B;
        double tt = 2.*tracking->ntraj.fTEnd - solver.t();
        Eigen::Matrix<double, 4, 1> x0;
        x0 << tracking->ntraj.fX[0](tt), tracking->ntraj.fX[1](tt),
              tracking->ntraj.fX[2](tt), tracking->ntraj.fX[3](tt);
        double u0 = tracking->ntraj.fU(tt);
        
        tracking->ltv_lqr.calcB(B.data(), x0.data(), u0);
        
        K = tracking->ltv_lqr.R.inverse()*B.transpose()*S;
        
        //std::cout << solver.t() << " " << K(0,0) << " " << K(0,1);
        //std::cout << " " << K(0,2) << " " << K(0,3) << std::endl;
        
        t.at(255-tidx) = tt;
        for(int i=0; i<4; i++) k[i].at(255-tidx) = K(0,i);
    }
    
    //std::cout << Eigen::Map<const Eigen::Matrix<double, 4, 4> >(solver.y()) << std::endl;
    
    for(int i=0; i<4; i++)
        tracking->gains.fK[i].init(gsl_interp_cspline, t, k[i]);
}

int main()
{
    using namespace GiNaC;
    struct AcrobotParam par;
    par.G = 9.81;
    par.M1 = .5;
    par.M2 = .2;
    par.LC = .3;
    par.L1 = .5;
    par.L2 = .6;
    par.I1 = 0.;
    par.I2 = 0.;
    par.GAMMA = 0.;
    
    Acrobot a(par);
    Lagrange l(a);
    Tracking tracking(l, a.u1);
    
    calcGainSchedule(&tracking);
    
    std::vector<symbol> q = l.q(), qdot = l.qdot();
    GiNaC::ex qddot = l.qdotdot();
    
    tracking.func = func_compiler::compile(l.qdotdot(), l.q(), l.qdot(), a.u1);
    
    const double x0_ini[] = { M_PI, 0.0, 0., 0. };  // q, qdot
    const double xbar_ini[] = { 0.2, 0.2, 0.0, 0.0 };
    const double x_ini[] = { x0_ini[0] + xbar_ini[0],
                             x0_ini[1] + xbar_ini[1],
                             x0_ini[2] + xbar_ini[2],
                             x0_ini[3] + xbar_ini[3] };
    const double tstep = 1./100.;
    int i;
    
    RawODESolver solver(4, calc_xdot, 0, x_ini, &tracking);
    RawODESolver solver2(4, calc_xbardot_linear, 0, xbar_ini, &tracking);
    
    std::cout << "#:1:q0" << std::endl;
    std::cout << "#:2:q1" << std::endl;
    std::cout << "#:3:qdot0" << std::endl;
    std::cout << "#:4:qdot1" << std::endl;
    
    std::cout << "#:5:q0_nom" << std::endl;
    std::cout << "#:6:q1_nom" << std::endl;
    std::cout << "#:7:qdot0_nom" << std::endl;
    std::cout << "#:8:qdot1_nom" << std::endl;
    std::cout << "#:9:u_nom" << std::endl;
    
    std::cout << "#:10:K0" << std::endl;
    std::cout << "#:11:K1" << std::endl;
    std::cout << "#:12:K2" << std::endl;
    std::cout << "#:13:K3" << std::endl;
    
    std::cout << "#:14:d_q0" << std::endl;
    std::cout << "#:15:d_q1" << std::endl;
    std::cout << "#:16:d_qdot0" << std::endl;
    std::cout << "#:17:d_qdot1" << std::endl;
    
    std::cout << "#:18:d_q0_lin" << std::endl;
    std::cout << "#:19:d_q1_lin" << std::endl;
    std::cout << "#:20:d_qdot0_lin" << std::endl;
    std::cout << "#:21:d_qdot1_lin" << std::endl;
    
    std::cout.precision(10);
    
    bool running = true;
    int i_end = tracking.ntraj.fTEnd / tstep;
    for(i=0; i<=i_end; i++) {
        const double t = i * tstep;
        if(running)
            solver.EvolveFwd(t);
        solver2.EvolveFwd(t);
        
        std::cout << t << " ";
        for(int i=0; i<4; i++)
            std::cout << solver.y()[i] << " ";
        
        for(int i=0; i<4; i++)
            std::cout << tracking.ntraj.fX[i](t) << " ";
        std::cout << tracking.ntraj.fU(t) << " ";
        
        for(int i=0; i<4; i++)
            std::cout << tracking.gains.fK[i](t) << " ";
        
        for(int i=0; i<4; i++)
            std::cout << (solver.y()[i] - tracking.ntraj.fX[i](t)) << " ";
        
        for(int i=0; i<4; i++)
            std::cout << solver2.y()[i] << " ";
        std::cout << std::endl;
        
        if(std::abs(solver.y()[0]) > 10. || std::abs(solver.y()[1]) > 10.)
            running = false;
    }
    
    return 0;
}

/* int _main()
{
    using namespace GiNaC;
    Acrobot a;
    Lagrange l(a);
    Tracking tracking(l, a.u1);
    
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> A;
    Eigen::Matrix<double, 4, 1> B;
    
    Eigen::Matrix<double, 4, 1> x0;
    x0 << 0, 0, 0, 0;
    
    tracking.ltv_lqr.calcA(A.data(), x0.data(), 0.);
    tracking.ltv_lqr.calcB(B.data(), x0.data(), 0.);
    
    std::cout << A << std::endl;
    std::cout << B << std::endl;
    
    return 0;
} */

/* int nomain()
{
    GiNaC::symbol x("x");
    GiNaC::matrix A(2, 2);
    
    A = 1, x,
        0, 4;
    
    typedef GiJIT::CodeGenV<GiJIT::Result, GiJIT::Number> test_compiler;
    test_compiler::func_t func = test_compiler::compile(A, x);
    
    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> foo;
    func(foo.data(), 5.);
    Eigen::Vector2d bar(1, 0);
    
    std::cout << foo << std::endl;
    
    return 0;
} */
