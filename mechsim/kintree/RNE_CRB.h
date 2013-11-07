#ifndef CW_RNE_CRB_H
#define CW_RNE_CRB_H

#include <Eigen/Dense>
#include "Spatial.h"
#include "KinChain2.h"

SpForce calc_react(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g = Eigen::Vector3d::Zero());

Eigen::VectorXd calc_u(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g = Eigen::Vector3d::Zero());
Eigen::MatrixXd calc_du_dq(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g = Eigen::Vector3d::Zero());
Eigen::MatrixXd calc_du_dqdot(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot);

Eigen::MatrixXd calc_M(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq);
Eigen::MatrixXd calc_dM_dq(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, unsigned int diff_id);

#endif
