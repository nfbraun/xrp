#include <iostream>
#include <iomanip>
#include "Cartwheel.h"
#include "DynTransform.h"
#include "DynInfo.h"

unsigned int col = 1;

void quatHeader(const std::string& name)
{
    std::cout << "#:" << col++ << ":" << name << "_w\n";
    std::cout << "#:" << col++ << ":" << name << "_x\n";
    std::cout << "#:" << col++ << ":" << name << "_y\n";
    std::cout << "#:" << col++ << ":" << name << "_z\n";
}

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

void printQuat(const Eigen::Quaterniond& q)
{
    std::cout << q.w() << " ";
    std::cout << q.x() << " ";
    std::cout << q.y() << " ";
    std::cout << q.z() << " ";
}

void printVect(const Eigen::Vector3d& v)
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
    
    std::cout << std::setprecision(15);
    
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":p_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":o_" << dofName(i) << "\n";
    for(unsigned int i=0; i<DOF_MAX; i++)
        std::cout << "#:" << col++ << ":ct_" << dofName(i) << "\n";
    
    vectHeader("rp");
    quatHeader("rq");
    vectHeader("rv");
    vectHeader("ra");
    
    vectHeader("lf");
    vectHeader("lt");
    vectHeader("rf");
    vectHeader("rt");
    
    scalarHeader("stance");
    
    // Wait for stance to toggle twice
    /* while(1) {
        sim.Advance();
        CartState state = sim.GetCurrentState();
        if(state.fDbg.stance == 1)
            break;
    }
    
    while(1) {
        sim.Advance();
        CartState state = sim.GetCurrentState();
        if(state.fDbg.stance == 0)
            break;
    } */
    
    sim.Advance();
    sim.Advance();
    
    // Print the third step
    unsigned int t = 0;
    while(1) {
        CartState state = sim.GetCurrentState();
        
        //if(state.fDbg.stance == 1)
        //    break;
        if(t > 10000)
            break;
        
        std::cout << t*sim.GetTimestep() << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.phi(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fJState.omega(i) << " ";
        
        for(unsigned int i=0; i<DOF_MAX; i++)
            std::cout << state.fCtrlTorques.t(i) << " ";
        
        printVect(state.fFState.pos(B_PELVIS));
        printQuat(state.fFState.rot(B_PELVIS));
        printVect(state.fFState.vel(B_PELVIS));
        printVect(state.fFState.avel(B_PELVIS));
        
        printVect(state.fLF);
        printVect(state.fLT);
        printVect(state.fRF);
        printVect(state.fRT);
        
        printScalar(state.fStance);
        
        std::cout << std::endl;
        
        sim.Advance();
        t++;
    }
    
    return 0;
}
