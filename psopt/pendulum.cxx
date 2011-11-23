/*
    Pendulum
    
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
   return tf;
} 

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters, 
                     adouble& time, adouble* xad, int iphase)
{
    return  0.0;
} 


//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void dae(adouble* derivatives, adouble* path, adouble* states, 
         adouble* controls, adouble* parameters, adouble& time, 
         adouble* xad, int iphase)
{
   adouble phidot, omegadot;

   adouble phi = states[ CINDEX(1) ];
   adouble omega = states[ CINDEX(2) ];

   adouble tau = controls[ CINDEX(1) ];

   phidot = omega;
   omegadot = -sin(phi) + tau;

   derivatives[ CINDEX(1) ] = phidot;
   derivatives[ CINDEX(2) ] = omegadot;

   // path[ CINDEX(1) ] = u*u; 

}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the events function ////////////////////////////
////////////////////////////////////////////////////////////////////////////

void events(adouble* e, adouble* initial_states, adouble* final_states, 
            adouble* parameters,adouble& t0, adouble& tf, adouble* xad, 
            int iphase) 

{
   adouble phi0 = initial_states[ CINDEX(1) ];
   adouble omega0 = initial_states[ CINDEX(2) ];
   adouble phif = final_states[ CINDEX(1) ];
   adouble omegaf = final_states[ CINDEX(2) ];

   e[ CINDEX(1) ] = phi0;
   e[ CINDEX(2) ] = omega0;
   e[ CINDEX(3) ] = phif;
   e[ CINDEX(4) ] = omegaf;

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

    problem.name        		= "Pendulum";
    problem.outfilename                 = "pendulum.txt";

////////////////////////////////////////////////////////////////////////////
////////////  Define problem level constants & do level 1 setup ////////////
////////////////////////////////////////////////////////////////////////////

    problem.nphases   			= 1;
    problem.nlinkages                   = 0;

    psopt_level1_setup(problem);

/////////////////////////////////////////////////////////////////////////////
/////////   Define phase related information & do level 2 setup  ////////////
/////////////////////////////////////////////////////////////////////////////

    problem.phases(1).nstates   		= 2;
    problem.phases(1).ncontrols 		= 1;
    problem.phases(1).nevents   		= 4;
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

    double phi0 = 0.0;
    double omega0 = 0.0;
    double phif = 3.141592;
    double omegaf = 0.0;

    problem.phases(1).bounds.lower.states(1) = -10.;
    problem.phases(1).bounds.lower.states(2) = -10.;

    problem.phases(1).bounds.upper.states(1) = 10.;
    problem.phases(1).bounds.upper.states(2) = 10.;

    problem.phases(1).bounds.lower.controls(1) = -.5;
    problem.phases(1).bounds.upper.controls(1) = .5;

    problem.phases(1).bounds.lower.events(1) = phi0;
    problem.phases(1).bounds.lower.events(2) = omega0;
    problem.phases(1).bounds.lower.events(3) = phif;
    problem.phases(1).bounds.lower.events(4) = omegaf;

    problem.phases(1).bounds.upper.events(1) = phi0;
    problem.phases(1).bounds.upper.events(2) = omega0;
    problem.phases(1).bounds.upper.events(3) = phif;
    problem.phases(1).bounds.upper.events(4) = omegaf;

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

    x_guess(1,colon()) = phi0*ones(1,nnodes);
    x_guess(2,colon()) = omega0*ones(1,nnodes);

    problem.phases(1).guess.controls       = zeros(ncontrols,nnodes);
    problem.phases(1).guess.states         = x_guess;
    problem.phases(1).guess.time           = linspace(0.0,2.0,nnodes);


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
    
    std::ofstream file("pendulum.dat");
    for(int i=1; i<=t.getm(); i++) {
        file << t.elem(1,i) << " ";
        file << x.elem(1,i) << " " << x.elem(2, i) << " ";
        //file << x.elem(3,i) << " " << x.elem(4, i) << " ";
        file << u.elem(1,i) << std::endl;
    }
    file.close();


////////////////////////////////////////////////////////////////////////////
///////////  Save solution data to files if desired ////////////////////////
////////////////////////////////////////////////////////////////////////////

    /* x.Save("x.dat");
    u.Save("u.dat");
    t.Save("t.dat");
    lambda.Save("lambda.dat");
    H.Save("H.dat"); */

////////////////////////////////////////////////////////////////////////////
///////////  Plot some results if desired (requires gnuplot) ///////////////
////////////////////////////////////////////////////////////////////////////

    /* plot(t,x,problem.name+": states", "time (s)", "states","x y v");

    plot(t,u,problem.name+": controls","time (s)", "controls", "u_1 u_2");

    plot(t,x,problem.name+": states", "time (s)", "states","x y v", 
                             "pdf", "brymr_states.pdf");

    plot(t,u,problem.name+": controls","time (s)", "controls", "u_1 u_2",
                             "pdf", "brymr_controls.pdf"); */
}

////////////////////////////////////////////////////////////////////////////
///////////////////////      END OF FILE     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////
