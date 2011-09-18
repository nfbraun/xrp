#include <iostream>
#include "Acrobot.h"

double angle(const Eigen::Vector3d& r)
{
    return atan2(r.x(), r.z());
}

double norm_angle(double x)
{
    while(x < 0.) x += 2.*M_PI;
    return fmod(x, 2.*M_PI);
}

int main()
{
    Acrobot sim;
    
    for(int t=0; t<100*Acrobot::STEP_PER_SEC; t++) {
        const AcroState* state = sim.GetState(t);
        
        std::cout << t*sim.GetTimestep() << " ";
        
        double theta1 = norm_angle(angle(state->fB1_pos));
        double theta2 = norm_angle(angle(state->fB2_pos - state->fB1_pos) - theta1);
        
        std::cout << theta1 << " " << theta2;
        
        std::cout << std::endl;
    }
    
    return 0;
}

