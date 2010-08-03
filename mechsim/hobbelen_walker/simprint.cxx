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
    
    for(int t=0; t<1000; t++) {
        const HobState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        
        double theta = GetAngle(state->fOULegQ.rot()) - HobbelenConst::GAMMA;
        
        //std::cout << state->dbg1;
        //std::cout << state->fOULegQ.avel().y() << " ";
        //std::cout << GetAngle(state->fOULegQ.rot()) - HobbelenConst::GAMMA << " ";
        //std::cout << GetAngle(state->fOLLegQ.rot()) - HobbelenConst::GAMMA << " ";
        std::cout << theta << " ";
        //std::cout << GetAngle(state->fBodyQ.rot()) << " ";
        // std::cout << GetAngle(state->fOFootQ.rot()) - HobbelenConst::GAMMA << " ";
        std::cout << theta - GetAngle(state->fIULegQ.rot()) - HobbelenConst::GAMMA << " ";
        std::cout << theta - GetAngle(state->fILLegQ.rot()) - HobbelenConst::GAMMA << " ";
        // std::cout << GetAngle(state->fIFootQ.rot()) - HobbelenConst::GAMMA << " ";
        std::cout << std::endl;
    }
    
    return 0;
}

