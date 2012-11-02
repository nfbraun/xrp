#include <iostream>
#include <cmath>
#include "ODEHelper.h"
#include "SolidBody.h"
#include "McGeer.h"

// Alternate computation of right foot center; does not require leg to be intact
Eigen::Vector3d GetFootCtr(const MGState& state)
{
    return state.fOLeg.fSPos + state.fOLeg.fSRot *
             ODE::VectorFromArray(dGeomGetOffsetPosition(state.fParent->fOuterFootG));
}

// Function for comparing single leg rolloff to Lagrangian solution
int main(int argc, char**)
{
    using Eigen::Vector3d;
    
    McGeer sim;
    
    const Vector3d par(cos(McGeer::GAMMA), 0., -sin(McGeer::GAMMA));
    const Vector3d nor(sin(McGeer::GAMMA), 0.,  cos(McGeer::GAMMA));
    
    const MGState state0 = sim.GetCurrentState();
    
    typedef Eigen::DiagonalMatrix<double, 3> Diag;
    
    SolidBody hip = SolidBody::SphereTotal(McGeer::M_H, .01, 
        state0.fOLeg.hipCoG());
    SolidBody thigh(McGeer::M_T, state0.fOLeg.thighCoG(),
        Diag(1., McGeer::M_T*McGeer::R_GYR_T, 1.));
    SolidBody shank(McGeer::M_S, state0.fOLeg.shankCoG(),
        Diag(1., McGeer::M_S*McGeer::R_GYR_S, 1.));
    
    SolidBody leg = combine(combine(hip, thigh), shank);
    
    Vector3d d = leg.cog() - GetFootCtr(state0);
    const double phi0 = atan2(d.x(), d.z());
    
    if(argc != 2) {
        std::cout << "m     = " << leg.m() << std::endl;
        std::cout << "R     = " << McGeer::R << std::endl;
        std::cout << "d     = " << d.norm() << std::endl;
        std::cout << "I     = " << leg.I()(1,1) << std::endl;
        std::cout << "gamma = " << McGeer::GAMMA << std::endl;
        std::cout << "phi0  = " << phi0 << std::endl;
    }
    
    const double d0 = GetFootCtr(state0).dot(par);
    
    if(argc == 2) {
        for(int t=0; t<sim.GetDefaultEndTime()/2; t++) {
            const MGState state = sim.GetCurrentState();
            
            /* double T =  .5 * McGeer::M_T * state->fRLeg.fTVel.mag2()
                      + .5 * McGeer::M_T * McGeer::R_GYR_T * state->fRLeg.fTOme.mag2()
                      + .5 * McGeer::M_S * state->fRLeg.fSVel.mag2()
                      + .5 * McGeer::M_S * McGeer::R_GYR_S * state->fRLeg.fSOme.mag2();
            double V = McGeer::M_T * Vector::dot(state->fRLeg.fTPos, Vector3::eZ)
                     + McGeer::M_S * Vector::dot(state->fRLeg.fSPos, Vector3::eZ);
                     
            // Alternate calculation
            double phiDot = state->fRLeg.fSOme.y();
            Vector3 cog = (McGeer::M_T * state->fRLeg.fTPos + McGeer::M_S * state->fRLeg.fSPos)
                               / (McGeer::M_T + McGeer::M_S);
            Vector3 vcog = phiDot*McGeer::R*par + 
                Vector::cross(phiDot*Vector3::eY, cog - GetFootCtr(state));
                
            double Tp = .5 * (McGeer::M_T + McGeer::M_S) * vcog.mag2()
                       + .5 * leg.I()(1,1) * phiDot * phiDot; */
            
            Vector3d center = GetFootCtr(state);
            double height = center.dot(nor);
            double dist = center.dot(par);
            std::cout << t*sim.GetTimestep() << " ";
            // std::cout << T << " " << Tp;
            // std::cout << T + V << " ";
            std::cout << (dist-d0)/McGeer::R + phi0 << " " << height << " ";
            std::cout << state.fOLeg.thighAng() - state.fOLeg.shankAng();
            std::cout << std::endl;
            
            sim.Advance();
        }
    }
    
    return 0;
}

