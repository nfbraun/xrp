#include <iostream>
#include <cmath>
#include "Hobbelen.h"

double GetAngle(const Rotation& rot)
{
    Vector3 zp = rot * Vector3::eZ;
    return atan2(zp.x(), zp.z());
}

int main()
{
    Hobbelen sim;
    
    // Output dataplot header
    std::cout << "#:1:Body CoG x\n";
    std::cout << "#:2:Body CoG y\n";
    std::cout << "#:3:Body CoG z\n";
    std::cout << "#:4:Body CoG x vel\n";
    std::cout << "#:5:Body CoG y vel\n";
    std::cout << "#:6:Body CoG z vel\n";
    
    std::cout << "#:7:Left hip angle\n";
    std::cout << "#:8:Right hip angle\n";
    std::cout << "#:9:Inter-leg angle\n";
    std::cout << "#:10:Desired inter-leg angle\n";
    
    std::cout << "#:11:Left knee angle\n";
    std::cout << "#:12:Right knee angle\n";
    std::cout << "#:13:Desired knee angle\n";
    
    std::cout << "#:14:Left ankle angle\n";
    std::cout << "#:15:Right ankle angle\n";
    
    std::cout << "#:16:Left tip clearance\n";
    std::cout << "#:17:Left heel clearance\n";
    std::cout << "#:18:Right tip clearance\n";
    std::cout << "#:19:Right heel clearance\n";
    
    std::cout << "#:20:Left leg state\n";
    std::cout << "#:21:Right leg state" << std::endl;
    
    // std::cout << "#:X:DEBUG: Inner leg angle\n";
    // std::cout << "#:X:DEBUG: Outer leg angle" << std::endl;
    
    for(int t=0; t<200; t++) {
        const HobState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        
        std::cout << state->fBodyQ.pos().x() << " ";
        std::cout << state->fBodyQ.pos().y() << " ";
        std::cout << state->fBodyQ.pos().z() << " ";
        std::cout << state->fBodyQ.vel().x() << " ";
        std::cout << state->fBodyQ.vel().y() << " ";
        std::cout << state->fBodyQ.vel().z() << " ";
        
        std::cout << state->fLHipAngle << " " << state->fRHipAngle << " ";
        std::cout << (state->fRHipAngle - state->fLHipAngle) << " ";
        std::cout << state->fDesiredInterLeg << " ";
        
        std::cout << state->fLKneeAngle << " " << state->fRKneeAngle << " ";
        std::cout << state->fDesiredKneeAngle << " ";
        
        std::cout << state->fLAnkleAngle << " " << state->fRAnkleAngle << " ";
        
        std::cout << state->fLTipClear << " " << state->fLHeelClear << " ";
        std::cout << state->fRTipClear << " " << state->fRHeelClear << " ";
        
        std::cout << state->fLLegState << " " << state->fRLegState << " ";
        
        // std::cout << GetAngle(state->fIULegQ.rot()) - HobbelenConst::GAMMA << " ";
        // std::cout << GetAngle(state->fOULegQ.rot()) - HobbelenConst::GAMMA << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}

