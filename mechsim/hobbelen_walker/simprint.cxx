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
    std::cout << "#:2:Body CoG z\n";
    std::cout << "#:3:Body CoG x vel\n";
    
    std::cout << "#:4:Inner hip angle\n";
    std::cout << "#:5:Outer hip angle\n";
    std::cout << "#:6:Inter-leg angle\n";
    std::cout << "#:7:Desired inter-leg angle\n";
    
    std::cout << "#:8:Inner knee angle\n";
    std::cout << "#:9:Outer knee angle\n";
    std::cout << "#:10:Inner ankle angle\n";
    std::cout << "#:11:Outer ankle angle\n";
    
    std::cout << "#:12:Inner tip clearance\n";
    std::cout << "#:13:Inner heel clearance\n";
    std::cout << "#:14:Outer tip clearance\n";
    std::cout << "#:15:Outer heel clearance\n";
    
    std::cout << "#:16:Inner leg state\n";
    std::cout << "#:17:Outer leg state" << std::endl;
    
    // std::cout << "#:18:DEBUG: Inner leg angle\n";
    // std::cout << "#:19:DEBUG: Outer leg angle" << std::endl;
    
    for(int t=0; t<200; t++) {
        const HobState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        
        std::cout << state->fBodyQ.pos().x() << " ";
        std::cout << state->fBodyQ.pos().z() << " ";
        std::cout << state->fBodyQ.vel().x() << " ";
        
        std::cout << state->fIHipAngle << " " << state->fOHipAngle << " ";
        std::cout << (state->fOHipAngle - state->fIHipAngle) << " ";
        std::cout << state->fDesiredInterLeg << " ";
        
        std::cout << state->fIKneeAngle << " " << state->fOKneeAngle << " ";
        std::cout << state->fIAnkleAngle << " " << state->fOAnkleAngle << " ";
        
        std::cout << state->fITipClear << " " << state->fIHeelClear << " ";
        std::cout << state->fOTipClear << " " << state->fOHeelClear << " ";
        
        std::cout << state->fILegState << " " << state->fOLegState << " ";
        
        // std::cout << GetAngle(state->fIULegQ.rot()) - HobbelenConst::GAMMA << " ";
        // std::cout << GetAngle(state->fOULegQ.rot()) - HobbelenConst::GAMMA << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}

