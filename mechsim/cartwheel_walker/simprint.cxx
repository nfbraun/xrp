#include "Cartwheel.h"
#include "DynTransform.h"

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
    std::cout << v.x() << " ";
    std::cout << v.y() << " ";
    std::cout << v.z() << " ";
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
    
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":p_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":o_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":t_" << dofName(i) << "\n";
    
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
    
    scalarHeader("sfwr"); // swing foot weight ratio
    scalarHeader("stance");
    scalarHeader("phi");
    
    scalarHeader("lfn");  // left foot normal force
    scalarHeader("lft");  // left foot tangential force
    scalarHeader("rfn");  // right foot normal force
    scalarHeader("rft");  // right foot tangential force
    
    for(int t=0; t<10000; t++) {
        CartState state = sim.GetCurrentState();
        
        std::cout << t*sim.GetTimestep() << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.phi(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.omega(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fTorques.t(i) << " ";
        
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
        
        printScalar(state.fDbg.StanceFootWeightRatio);
        printScalar(state.fDbg.stance);
        printScalar(state.fDbg.phi);
        
        printScalar(state.fDbg.lFootNF);
        printScalar(state.fDbg.lFootTF);
        printScalar(state.fDbg.rFootNF);
        printScalar(state.fDbg.rFootTF);
        
        std::cout << std::endl;
        
        sim.Advance();
    }
    
    return 0;
}
