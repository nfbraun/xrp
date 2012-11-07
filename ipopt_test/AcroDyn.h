#ifndef INCL_ACRODYN_H
#define INCL_ACRODYN_H

#include <Eigen/Dense>
#include <coin/IpTNLP.hpp>

using Ipopt::Number;
using Ipopt::Index;

const Index N_dof = 2;
const Index N_ctrl = 1;

typedef Eigen::Matrix<Number, N_dof, 1> Vector_Ndof;
typedef Eigen::Matrix<Number, N_ctrl, 1> Vector_Nctrl;

class AcroDyn {
  public:
    struct DResult {
        Vector_Ndof f;
        Vector_Ndof df[2*N_dof+N_ctrl];
        Vector_Ndof ddf[2*N_dof+N_ctrl][2*N_dof+N_ctrl];
    };
    
    static DResult qddot_full(const Vector_Ndof& q, const Vector_Ndof& qdot,
                              const Vector_Nctrl& u);
};

#endif
