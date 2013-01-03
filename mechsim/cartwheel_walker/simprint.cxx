#include "Cartwheel.h"

int main(int argc, char** argv)
{
    Cartwheel sim(2500, 1);
    
    unsigned int col = 1;
    std::cout << "#:" << col++ << ":swp_x\n";
    std::cout << "#:" << col++ << ":swp_y\n";
    std::cout << "#:" << col++ << ":swp_z\n";
    std::cout << "#:" << col++ << ":swv_x\n";
    std::cout << "#:" << col++ << ":swv_y\n";
    std::cout << "#:" << col++ << ":swv_z\n";
    
    std::cout << "#:" << col++ << ":sfwr\n";
    
    for(unsigned int i=LH0; i<=RA1; i++)
        std::cout << "#:" << col++ << ":p_" << PhiNames[i] << "\n";
    for(unsigned int i=LH0; i<=RA1; i++)
        std::cout << "#:" << col++ << ":o_" << PhiNames[i] << "\n";
    for(unsigned int i=LH0; i<=RA1; i++)
        std::cout << "#:" << col++ << ":t_" << PhiNames[i] << "\n";
    
    for(int t=0; t<10000; t++)
        sim.Advance();
    
    for(int t=0; t<10000; t++) {
        CartState state = sim.GetCurrentState();
        
        std::cout << t*sim.GetTimestep() << " ";
        
        std::cout << state.fDbg.desSwingPos.x << " ";
        std::cout << state.fDbg.desSwingPos.y << " ";
        std::cout << state.fDbg.desSwingPos.z << " ";
        std::cout << state.fDbg.desSwingVel.x << " ";
        std::cout << state.fDbg.desSwingVel.y << " ";
        std::cout << state.fDbg.desSwingVel.z << " ";
        
        std::cout << state.fDbg.StanceFootWeightRatio << " ";
        
        for(unsigned int i=LH0; i <= RA1; i++)
            std::cout << state.fRobot.phi[i] << " ";
        
        for(unsigned int i=LH0; i <= RA1; i++)
            std::cout << state.fRobot.omega[i] << " ";
        
        for(unsigned int i=LH0; i <= RA1; i++)
            std::cout << state.fTorques[i] << " ";
        
        std::cout << std::endl;
        
        sim.Advance();
    }
    
    return 0;
}
