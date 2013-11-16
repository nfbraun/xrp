#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "CSVFile.h"
#include "Robot.h"
#include "RNE_CRB.h"

int main()
{
    const unsigned int stIdx = 62; // FIXME
    
    CSVFile csv("step.dat");
    if(csv.fail()) {
        throw std::runtime_error("Failed to open step.dat");
    }
    
    const char* names[] = { "LHZ", "LHY", "LHX", "LKY", "LAY", "LAX", "RHZ", "RHY", "RHX", "RKY", "RAY", "RAX" };
    assert(sizeof(names)/sizeof(names[0]) == Robot::N_DOF);
    
    for(unsigned int i=0; i<Robot::N_DOF; i++)
        std::cout << "#:" << i+1 << ":p_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<Robot::N_DOF; i++)
        std::cout << "#:" << i+1+Robot::N_DOF << ":o_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<Robot::N_DOF; i++)
        std::cout << "#:" << i+1+2*Robot::N_DOF << ":t_" << names[i] << "_ana" << std::endl;
    
    unsigned int t_idx_end = csv.data().size() - 2;
    
    // Will fail for variable intervals!
    const double dt = csv.data().at(1).at(0) - csv.data().at(0).at(0);
    
    for(unsigned int t_idx=1; t_idx<t_idx_end; t_idx++) {
        double t = csv.data().at(t_idx).at(0);
        
        std::cout << t << " ";
        
        Eigen::VectorXd q(Robot::N_DOF);
        Eigen::VectorXd qdot(Robot::N_DOF);
        Eigen::VectorXd qddot(Robot::N_DOF);
        
        const unsigned int dofOrder_L[] = { 5, 4, 3, 2, 1, 0, 6, 7, 8, 9, 10, 11 };
        const unsigned int invDofOrder_L[] = { 5, 4, 3, 2, 1, 0, 6, 7, 8, 9, 10, 11 };
        const unsigned int dofOrder_R[] = { 11, 10, 9, 8, 7, 6, 0, 1, 2, 3, 4, 5 };
        const unsigned int invDofOrder_R[] = { 6, 7, 8, 9, 10, 11, 5, 4, 3, 2, 1, 0 };
        const unsigned int* dofOrder;
        const unsigned int* invDofOrder;
        
        int stance = csv.data().at(t_idx).at(stIdx);
        if(stance == 0) {
            dofOrder = dofOrder_L;
            invDofOrder = invDofOrder_L;
        } else {
            dofOrder = dofOrder_R;
            invDofOrder = invDofOrder_R;
        }
        
        for(unsigned int dof_idx=0; dof_idx<Robot::N_DOF; dof_idx++) {
            double q_ = csv.data().at(t_idx).at(dofOrder[dof_idx]+1);
            double q_plus = csv.data().at(t_idx+1).at(dofOrder[dof_idx]+1);
            double q_minus = csv.data().at(t_idx-1).at(dofOrder[dof_idx]+1);
            
            double qdot_ = csv.data().at(t_idx).at(dofOrder[dof_idx]+1+Robot::N_DOF);
            double qdot_plus = csv.data().at(t_idx+1).at(dofOrder[dof_idx]+1+Robot::N_DOF);
            double qdot_minus = csv.data().at(t_idx-1).at(dofOrder[dof_idx]+1+Robot::N_DOF);
            
            q(dof_idx) = q_;
            qdot(dof_idx) = qdot_;
            qddot(dof_idx) = (qdot_plus - qdot_minus) / (2. * dt);
            //qddot(dof_idx) = (q_plus + q_minus - 2*q_) / (dt * dt);
        }
        
        const Eigen::Vector3d g(0., 0., -9.81);
        
        Robot rob(stance);
        Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
        
        Eigen::VectorXd u(Robot::N_DOF);
        u = calc_u(rob, expSq, qdot, qddot, g);
        
        for(unsigned int dof_idx=0; dof_idx<Robot::N_DOF; dof_idx++)
            std::cout << q(invDofOrder[dof_idx]) << " ";
        for(unsigned int dof_idx=0; dof_idx<Robot::N_DOF; dof_idx++)
            std::cout << qdot(invDofOrder[dof_idx]) << " ";
        for(unsigned int dof_idx=0; dof_idx<Robot::N_DOF; dof_idx++)
            std::cout << u(invDofOrder[dof_idx]) << " ";
        
        std::cout << std::endl;
    }
    
    return 0;
}
