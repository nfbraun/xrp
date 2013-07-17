#ifndef CW_RNE_CRB_H
#define CW_RNE_CRB_H

#include <Eigen/Dense>

Eigen::VectorXd RNE(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot);
Eigen::MatrixXd CRB(const Eigen::VectorXd& q);

#endif
