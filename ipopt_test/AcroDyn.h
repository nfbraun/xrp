#ifndef INCL_ACRODYN_H
#define INCL_ACRODYN_H

#include <Eigen/Dense>
#include <coin/IpTNLP.hpp>

using Ipopt::Number;

typedef Eigen::Matrix<Number, 2, 1> Vector2N;

class AcroDyn {
  public:
    struct DResult {
        Vector2N f;
        Vector2N df[5];
        Vector2N ddf[5][5];
    };
    
    static DResult qddot_full(const Vector2N& q, const Vector2N& qdot, Number u);
};

#endif
