#include "Cartwheel.h"
#include "DynTransform.h"
#include "DynInfo.h"

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

void printVect(const Eigen::Vector3d& v)
{
    std::cout << v.x() << " ";
    std::cout << v.y() << " ";
    std::cout << v.z() << " ";
}

template<typename T>
void printScalar(T x)
{
    std::cout << x << " ";
}

int main(int argc, char** argv)
{
    Cartwheel sim(2500, 1);
    
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":p_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":o_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":ct_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":ft_" << dofName(i) << "\n";
    
    vectHeader("swp");    // desired swing foot pos
    vectHeader("swv");    // desired swing foot velocity
    scalarHeader("oc");   // CoM offset coronal
    scalarHeader("vc");   // CoM velocity coronal
    scalarHeader("vs");   // CoM velocity sagittal
    scalarHeader("doc");  // desired CoM offset coronal
    scalarHeader("dvc");  // desired CoM velocity coronal
    scalarHeader("dvs");  // desired CoM velocity sagittal
    vectHeader("vrf");    // virtual root force
    vectHeader("vrt");    // virtual root torque
    
    scalarHeader("stance");
    scalarHeader("phi");
    
    scalarHeader("lfn");  // left foot normal force
    scalarHeader("lft");  // left foot tangential force
    scalarHeader("rfn");  // right foot normal force
    scalarHeader("rft");  // right foot tangential force
    
    scalarHeader("lcm");  // left foot contact mask
    scalarHeader("rcm");  // right foot contact mask
    
    vectHeader("lcop");   // left foot center of pressure
    vectHeader("rcop");   // right foot center of pressure
    
    scalarHeader("Ekin");
    scalarHeader("Epot");
    scalarHeader("Pint");
    scalarHeader("Etot");
    
    vectHeader("lF");  // total reaction force on left foot
    vectHeader("lT");
    vectHeader("rF");  // total reaction force on right foot
    vectHeader("rT");
    
    vectHeader("stF"); // predicted reaction force on stance foot
    vectHeader("stT");
    
    scalarHeader("vrfg"); // virtual root force gain
    
    // Data is irregular for the first two timesteps
    sim.Advance();
    
    for(int t=2; t<10000; t++) {
        sim.Advance();
        
        CartState state = sim.GetCurrentState();
        
        std::cout << t*sim.GetTimestep() << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.phi(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.omega(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fCtrlTorques.t(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fFiltTorques.t(i) << " ";
        
        printVect(state.fDbg.desSwingPos);
        printVect(state.fDbg.desSwingVel);
        
        printScalar(state.fDbg.offCoronal);
        printScalar(state.fDbg.velCoronal);
        printScalar(state.fDbg.velSagittal);
        printScalar(state.fDbg.desOffCoronal);
        printScalar(state.fDbg.desVelCoronal);
        printScalar(state.fDbg.desVelSagittal);
        
        printVect(state.fDbg.virtualRootForce);
        printVect(state.fDbg.virtualRootTorque);
        
        printScalar(state.fDbg.stance);
        printScalar(state.fDbg.phi);
        
        printScalar(state.fDbg.lFootNF);
        printScalar(state.fDbg.lFootTF);
        printScalar(state.fDbg.rFootNF);
        printScalar(state.fDbg.rFootTF);
        
        printScalar(state.fLContacts);
        printScalar(state.fRContacts);
        
        printVect(state.fDbg.lCoP);
        printVect(state.fDbg.rCoP);
        
        double Ek = Ekin(state.fFState);
        double Ep = Epot(state.fFState) - Epot0();
        
        printScalar(Ek);
        printScalar(Ep);
        printScalar(state.fPint);
        printScalar(Ek + Ep - state.fPint);
        
        printVect(state.fLF);
        printVect(state.fLT);
        printVect(state.fRF);
        printVect(state.fRT);
        
        printVect(state.fStF_pred);
        printVect(state.fStT_pred);
        
        printScalar(state.fDbg.vrfGain);
        
        std::cout << std::endl;
    }
    
    return 0;
}
