#include <iostream>
#include <cmath>
#include "SolidBody.h"
#include "McGeer.h"

// Alternate computation of right foot center; does not require leg to be intact
Vector3 GetFootCtr(const MGState* state)
{
    return state->fRLeg.fSPos + state->fRLeg.fSRot *
                Vector3(dGeomGetOffsetPosition(state->fParent->fRightFootG));
}

// Function for comparing single leg rolloff to Lagrangian solution
int main(int argc, char**)
{
    McGeer sim;
    
    const Vector3 par(cos(McGeer::GAMMA), 0., -sin(McGeer::GAMMA));
    const Vector3 nor(sin(McGeer::GAMMA), 0.,  cos(McGeer::GAMMA));
    
    const MGState* state0 = sim.GetState(0);
    
    SolidBody hip = SolidBody::SphereTotal(McGeer::M_H, .01, 
        state0->fRLeg.hipCoG());
    SolidBody thigh(McGeer::M_T, state0->fRLeg.thighCoG(),
        Matrix33::Diag(1., McGeer::M_T*McGeer::R_GYR_T, 1.));
    SolidBody shank(McGeer::M_S, state0->fRLeg.shankCoG(),
        Matrix33::Diag(1., McGeer::M_S*McGeer::R_GYR_S, 1.));
    
    SolidBody leg = combine(combine(hip, thigh), shank);
    
    Vector3 d = leg.cog() - GetFootCtr(state0);
    const double phi0 = atan2(d.x(), d.z());
    
    if(argc != 2) {
        std::cout << "m     = " << leg.m() << std::endl;
        std::cout << "R     = " << McGeer::R << std::endl;
        std::cout << "d     = " << d.mag() << std::endl;
        std::cout << "I     = " << leg.I()(1,1) << std::endl;
        std::cout << "gamma = " << McGeer::GAMMA << std::endl;
        std::cout << "phi0  = " << phi0 << std::endl;
    }
    
    const double d0 = Vector::dot(GetFootCtr(state0), par);
    
    if(argc == 2) {
        for(int t=0; t<sim.GetDefaultEndTime()/2; t++) {
            const MGState* state = sim.GetState(t);
            
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
            
            Vector3 center = GetFootCtr(state);
            double height = Vector::dot(center, nor);
            double dist = Vector::dot(center, par);
            std::cout << t*sim.GetTimestep() << " ";
            // std::cout << T << " " << Tp;
            // std::cout << T + V << " ";
            std::cout << (dist-d0)/McGeer::R + phi0 << " " << height << " ";
            std::cout << state->fRLeg.thighAng() - state->fRLeg.shankAng();
            std::cout << std::endl;
        }
    }
    
    return 0;
}

