#include "Cartwheel.h"

unsigned int col = 1;

void vectHeader(const std::string& name)
{
    std::cout << "#:" << col++ << ":" << name << "_x\n";
    std::cout << "#:" << col++ << ":" << name << "_y\n";
    std::cout << "#:" << col++ << ":" << name << "_z\n";
}

void scalarHeader(const std::string& name)
{
    std::cout << "#:" << col++ << ":" << name << "\n";
}

void printVect(const Vector3d& v)
{
    std::cout << v.x << " ";
    std::cout << v.y << " ";
    std::cout << v.z << " ";
}

void printScalar(double x)
{
    std::cout << x << " ";
}

void printScalar(int x)
{
    std::cout << x << " ";
}

int main(int argc, char** argv)
{
    Cartwheel sim(2500, 1);
    
    vectHeader("swp");    // desired swing foot pos
    vectHeader("swv");    // desired swing foot velocity
    vectHeader("vrf");    // virtual root force
    vectHeader("vrt");    // virtual root torque
    
    scalarHeader("sfwr"); // swing foot weight ratio
    scalarHeader("stance");
    scalarHeader("phi");
    
    scalarHeader("lfn");  // left foot normal force
    scalarHeader("lft");  // left foot tangential force
    scalarHeader("rfn");  // right foot normal force
    scalarHeader("rft");  // right foot tangential force
    
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
        
        printVect(state.fDbg.desSwingPos);
        printVect(state.fDbg.desSwingVel);
        printVect(state.fDbg.virtualRootForce);
        printVect(state.fDbg.virtualRootTorque);
        
        std::cout << state.fDbg.StanceFootWeightRatio << " ";
        std::cout << state.fDbg.stance << " ";
        std::cout << state.fDbg.phi << " ";
        
        printScalar(state.fDbg.lFootNF);
        printScalar(state.fDbg.lFootTF);
        printScalar(state.fDbg.rFootNF);
        printScalar(state.fDbg.rFootTF);
        
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
