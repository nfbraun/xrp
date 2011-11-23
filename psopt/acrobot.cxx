/*
    Acrobot
    
    This file is derived from PSOPT/examples/user/user.cxx
    PSOPT is distributed under the terms of the GNU Lesser
    General Public License (LGPL).
*/
#include <fstream>
#include "psopt.h"

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the end point (Mayer) cost function //////////
//////////////////////////////////////////////////////////////////////////

adouble endpoint_cost(adouble* initial_states, adouble* final_states, 
                      adouble* parameters,adouble& t0, adouble& tf, 
                      adouble* xad, int iphase)
{
    return 0.;
} 

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters, 
                     adouble& time, adouble* xad, int iphase)
{
    adouble phi1 = states[ CINDEX(1) ];
    adouble phi2 = states[ CINDEX(2) ];

    adouble tau = controls[ CINDEX(1) ];
    
    return phi1*phi1 + phi2*phi2 + 100.*tau*tau;
} 

namespace SystemConst {
    const double G = 9.81;
    const double M1 = .5;
    const double M2 = .2;
    const double LC = .3;
    const double L1 = .5;
    const double L2 = .6;
    const double I1 = 0.;
    const double I2 = 0.;
    // const double GAMMA = 0.;
}

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void dae(adouble* derivatives, adouble* path, adouble* states, 
         adouble* controls, adouble* parameters, adouble& time, 
         adouble* xad, int iphase)
{
    using namespace SystemConst;

    adouble theta1 = states[ CINDEX(1) ];
    adouble theta2 = states[ CINDEX(2) ];
    adouble omega1 = states[ CINDEX(3) ];
    adouble omega2 = states[ CINDEX(4) ];

    adouble tau = controls[ CINDEX(1) ];

    adouble m11 = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(theta2) + I1 + I2;
    adouble m12 = M2*(L2*L2 + L1*L2*cos(theta2)) + I2;
    adouble m21 = m12;
    adouble m22 = M2*L2*L2 + I2;

    adouble det_m = m11*m22 - m12*m21;

    adouble mi11 = m22 / det_m;
    adouble mi12 = -m12 / det_m;
    adouble mi21 = -m21 / det_m;
    adouble mi22 = m11 / det_m;
    
    adouble c1 = -M2*L1*L2*sin(theta2)*omega2*(2*omega1 + omega2);
    adouble c2 = M2*L1*L2*sin(theta2)*omega1*omega1;
    
    adouble dV_dtheta1 = -(M1*LC + M2*L1)*G*sin(theta1)
        - M2*L2*G*sin(theta1 + theta2);
    adouble dV_dtheta2 = -M2*L2*G*sin(theta1 + theta2);

    //adouble c1 = omega2 * (2.*omega1+omega2) * sin(phi2)
    //  + 2*sin(phi1) + sin(phi1+phi2);
    //adouble c2 = -omega1*omega1*sin(phi2) + sin(phi1+phi2);

    adouble a1 = mi11 * (-c1 - dV_dtheta1) + mi12 * (-c2 - dV_dtheta2);
    adouble a2 = mi21 * (-c1 - dV_dtheta1) + mi22 * (-c2 - dV_dtheta2);

    adouble b1 = mi12 * tau;
    adouble b2 = mi22 * tau;

    adouble theta1dot, theta2dot, omega1dot, omega2dot;
    theta1dot = omega1;
    theta2dot = omega2;
    omega1dot = a1 + b1;
    omega2dot = a2 + b2;

    derivatives[ CINDEX(1) ] = theta1dot;
    derivatives[ CINDEX(2) ] = theta2dot;
    derivatives[ CINDEX(3) ] = omega1dot;
    derivatives[ CINDEX(4) ] = omega2dot;

    // path[ CINDEX(1) ] = u*u; 

}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the events function ////////////////////////////
////////////////////////////////////////////////////////////////////////////

void events(adouble* e, adouble* initial_states, adouble* final_states, 
            adouble* parameters,adouble& t0, adouble& tf, adouble* xad, 
            int iphase) 

{
   e[ CINDEX(1) ] = initial_states[ CINDEX(1) ];
   e[ CINDEX(2) ] = initial_states[ CINDEX(2) ];
   e[ CINDEX(3) ] = initial_states[ CINDEX(3) ];
   e[ CINDEX(4) ] = initial_states[ CINDEX(4) ];
   
   e[ CINDEX(5) ] = final_states[ CINDEX(1) ];
   e[ CINDEX(6) ] = final_states[ CINDEX(2) ];
   e[ CINDEX(7) ] = final_states[ CINDEX(3) ];
   e[ CINDEX(8) ] = final_states[ CINDEX(4) ];

}



///////////////////////////////////////////////////////////////////////////
///////////////////  Define the phase linkages function ///////////////////
///////////////////////////////////////////////////////////////////////////

void linkages( adouble* linkages, adouble* xad)
{
  // No linkages as this is a single phase problem
}



////////////////////////////////////////////////////////////////////////////
///////////////////  Define the main routine ///////////////////////////////
////////////////////////////////////////////////////////////////////////////

int main(void)
{

////////////////////////////////////////////////////////////////////////////
///////////////////  Declare key structures ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    Alg  algorithm;
    Sol  solution;
    Prob problem;

////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem name  ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.name        		= "Acrobot";
    problem.outfilename                 = "acrobot.txt";

////////////////////////////////////////////////////////////////////////////
////////////  Define problem level constants & do level 1 setup ////////////
////////////////////////////////////////////////////////////////////////////

    problem.nphases   			= 1;
    problem.nlinkages                   = 0;

    psopt_level1_setup(problem);

/////////////////////////////////////////////////////////////////////////////
/////////   Define phase related information & do level 2 setup  ////////////
/////////////////////////////////////////////////////////////////////////////

    problem.phases(1).nstates   		= 4;
    problem.phases(1).ncontrols 		= 1;
    problem.phases(1).nevents   		= 8;
    problem.phases(1).npath     		= 0;
    problem.phases(1).nodes                     = "[40]"; 

    psopt_level2_setup(problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////////////  Declare DMatrix objects to store results //////////////
////////////////////////////////////////////////////////////////////////////

    DMatrix x, u, t;
    DMatrix lambda, H;

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter problem bounds information //////////////////////
////////////////////////////////////////////////////////////////////////////

    double phi1_0 = 3.1415;
    double phi2_0 = 0.;

    problem.phases(1).bounds.lower.states(1) = -10.;
    problem.phases(1).bounds.lower.states(2) = -10.;
    problem.phases(1).bounds.lower.states(3) = -20.;
    problem.phases(1).bounds.lower.states(4) = -20.;

    problem.phases(1).bounds.upper.states(1) = 10.;
    problem.phases(1).bounds.upper.states(2) = 10.;
    problem.phases(1).bounds.upper.states(3) = 20.;
    problem.phases(1).bounds.upper.states(4) = 20.;

    problem.phases(1).bounds.lower.controls(1) = -5.;
    problem.phases(1).bounds.upper.controls(1) = 5.;

    problem.phases(1).bounds.lower.events(1) = phi1_0;
    problem.phases(1).bounds.lower.events(2) = phi2_0;
    problem.phases(1).bounds.lower.events(3) = 0.;
    problem.phases(1).bounds.lower.events(4) = 0.;
    problem.phases(1).bounds.lower.events(5) = 0.;
    problem.phases(1).bounds.lower.events(6) = 0.;
    problem.phases(1).bounds.lower.events(7) = 0.;
    problem.phases(1).bounds.lower.events(8) = 0.;

    problem.phases(1).bounds.upper.events(1) = phi1_0;
    problem.phases(1).bounds.upper.events(2) = phi2_0;
    problem.phases(1).bounds.upper.events(3) = 0.;
    problem.phases(1).bounds.upper.events(4) = 0.;
    problem.phases(1).bounds.upper.events(5) = 0.;
    problem.phases(1).bounds.upper.events(6) = 0.;
    problem.phases(1).bounds.upper.events(7) = 0.;
    problem.phases(1).bounds.upper.events(8) = 0.;


    //problem.phases(1).bounds.upper.path(1) = 1.0;
    //problem.phases(1).bounds.lower.path(1) = 1.0;



    problem.phases(1).bounds.lower.StartTime    = 0.0;
    problem.phases(1).bounds.upper.StartTime    = 0.0;

    problem.phases(1).bounds.lower.EndTime      = 0.0;
    problem.phases(1).bounds.upper.EndTime      = 20.0;



////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem functions  ///////////////////////////
////////////////////////////////////////////////////////////////////////////


    problem.integrand_cost 	= &integrand_cost;
    problem.endpoint_cost 	= &endpoint_cost;
    problem.dae             	= &dae;
    problem.events 		= &events;
    problem.linkages		= &linkages;

////////////////////////////////////////////////////////////////////////////
///////////////////  Define & register initial guess ///////////////////////
////////////////////////////////////////////////////////////////////////////

    int nnodes    			= problem.phases(1).nodes(1);
    int ncontrols                       = problem.phases(1).ncontrols;
    int nstates                         = problem.phases(1).nstates;

    DMatrix x_guess    =  zeros(nstates,nnodes);

    x_guess(1,colon()) = phi1_0*ones(1,nnodes);
    x_guess(2,colon()) = phi2_0*ones(1,nnodes);
    x_guess(3,colon()) = 0.*ones(1,nnodes);
    x_guess(4,colon()) = 0.*ones(1,nnodes);

    problem.phases(1).guess.controls       = zeros(ncontrols,nnodes);
    problem.phases(1).guess.states         = x_guess;
    problem.phases(1).guess.time           = linspace(0.0,0.5,nnodes);


////////////////////////////////////////////////////////////////////////////
///////////////////  Enter algorithm options  //////////////////////////////
////////////////////////////////////////////////////////////////////////////


    algorithm.nlp_iter_max                = 1000;
    algorithm.nlp_tolerance               = 1.e-4;
    algorithm.nlp_method                  = "IPOPT";
    algorithm.scaling                     = "automatic";
    algorithm.derivatives                 = "automatic";
    algorithm.mesh_refinement             = "automatic";
    algorithm.collocation_method = "trapezoidal";
//    algorithm.defect_scaling = "jacobian-based";
    algorithm.ode_tolerance               = 1.e-6;



////////////////////////////////////////////////////////////////////////////
///////////////////  Now call PSOPT to solve the problem   /////////////////
////////////////////////////////////////////////////////////////////////////

    psopt(solution, problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////  Extract relevant variables from solution structure   //////////
////////////////////////////////////////////////////////////////////////////


    x      = solution.get_states_in_phase(1);
    u      = solution.get_controls_in_phase(1);
    t      = solution.get_time_in_phase(1);
    lambda = solution.get_dual_costates_in_phase(1);
    H      = solution.get_dual_hamiltonian_in_phase(1);


////////////////////////////////////////////////////////////////////////////
///////////  Save solution data to files if desired ////////////////////////
////////////////////////////////////////////////////////////////////////////

    x.Save("x.dat");
    u.Save("u.dat");
    t.Save("t.dat");
    lambda.Save("lambda.dat");
    H.Save("H.dat");
    
    std::ofstream file("acrobot.dat");
    for(int i=1; i<=t.getm(); i++) {
        file << t.elem(1,i) << " ";
        file << x.elem(1,i) << " " << x.elem(2, i) << " ";
        file << x.elem(3,i) << " " << x.elem(4, i) << " ";
        file << u.elem(1,i) << std::endl;
    }
    file.close();

////////////////////////////////////////////////////////////////////////////
///////////  Plot some results if desired (requires gnuplot) ///////////////
////////////////////////////////////////////////////////////////////////////

    plot(t,x,problem.name+": states", "time (s)", "states","x y v");

    plot(t,u,problem.name+": controls","time (s)", "controls", "u_1 u_2");

    plot(t,x,problem.name+": states", "time (s)", "states","x y v", 
                             "pdf", "brymr_states.pdf");

    plot(t,u,problem.name+": controls","time (s)", "controls", "u_1 u_2",
                             "pdf", "brymr_controls.pdf");
}

////////////////////////////////////////////////////////////////////////////
///////////////////////      END OF FILE     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////
