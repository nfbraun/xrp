#include "DoubleInt.h"
#include "coin/IpIpoptApplication.hpp"
#include <iostream>

using Ipopt::SmartPtr;

int main()
{
    SmartPtr<Ipopt::TNLP> mynlp = new DoubleInt();
    SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    
    app->Options()->SetNumericValue("tol", 1e-9);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("linear_solver", "mumps");
    // app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    // app->Options()->SetStringValue("derivative_test", "first-order");
    
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if(status != Ipopt::Solve_Succeeded) {
        std::cout << "*** Failed to initialize solver." << std::endl;
        return status;
    }
    
    status = app->OptimizeTNLP(mynlp);
    if(status == Ipopt::Solve_Succeeded)
        std::cout << "*** Solver succeeded" << std::endl;
    else
        std::cout << "*** Solver failed" << std::endl;
    
    return status;
}
