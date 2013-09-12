#ifndef MSIM_KINCHAIN2_H
#define MSIM_KINCHAIN2_H

#include <Eigen/Dense>
#include "SE3Tr.h"
#include "Spatial.h"

/***
A kinematic chain is assumed to have the following structure:

FIXED - (J0) - (B0) - (J1) - (B1) - ... - (Jn-1) - (Bn-1)

This implies that the number of joints and the number of rigid bodies must be
equal.

Each joint can contain an arbitrary number of degrees of freedom.

***/

class KinChain2 {
  public:
    // Number of joints (equals number of rigid bodies)
    virtual unsigned int nJoints() const = 0;
    
    // Total number of degrees of freedom
    virtual unsigned int nDof() const = 0;
    
    // Number of degrees of freedom for joint i
    virtual unsigned int nJntDof(unsigned int i) const = 0;
    
    // Motion direction for DoF i (assumed to be constant)
    virtual SpMot S(unsigned int i) const = 0;
    
    // Spatial inertia for body i (assumed to be constant)
    virtual SpInertia I(unsigned int i) const = 0;
    
    // Affine transform associated with rigid body i (assumed to be constant)
    virtual SE3Tr bodyT(unsigned int i) const = 0;
};

#endif
