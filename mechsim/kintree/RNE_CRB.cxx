#include "RNE_CRB.h"
#include <Eigen/Dense>
#include "Spatial.h"
#include "../cartwheel_walker/StaticRobotInfo.h"

Eigen::VectorXd RNE(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot)
{
    Eigen::VectorXd C(6);
    C.setZero();
    
    SpMot S[6] = {
        SpMot(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.))
    };
    
    SE3Tr Tr[6] = {
        SE3Tr::RotZ(q(0)),
        SE3Tr::RotY(q(1)),
        SE3Tr::RotX(q(2)),
        SE3Tr::RotY(q(3)),
        SE3Tr::RotY(q(4)),
        SE3Tr::RotX(q(5))
    };
    
    SpInertia I[3] = {
        SpInertia(rbMass(B_R_THIGH),
                  Eigen::Vector3d(0., 0., -CharacterConst::thighSizeZ/2.),
                  rbMOI(B_R_THIGH).asDiagonal()),
        SpInertia(rbMass(B_R_SHANK),
                  Eigen::Vector3d(0., 0., -CharacterConst::shankSizeZ/2.),
                  rbMOI(B_R_SHANK).asDiagonal()),
        SpInertia(rbMass(B_R_FOOT),
                  Eigen::Vector3d(CharacterConst::footPosX, 0., -CharacterConst::footSizeZ/2.),
                  rbMOI(B_R_FOOT).asDiagonal())
    };
    
    SE3Tr T[3] = {
        SE3Tr::Trans(0., 0., -CharacterConst::thighSizeZ),
        SE3Tr::Trans(0., 0., -CharacterConst::shankSizeZ),
        SE3Tr::Identity()
    };
    
    int dof_max[4] = { -1, 2, 3, 5 };
    
    SpMot v[3];
    SpMot a[3];
    
    using namespace CharacterConst;
    
    SpMot v_i = SpMot::Zero();
    SpMot a_i = SpMot::Zero();
    
    int body_id = 0;
    int dof_id = 0;
    
    /*** Velocity/acceleration calculation ***/
    while(1) {
        for(; dof_id <= dof_max[body_id+1]; dof_id++) {
            v_i = v_i.tr(Tr[dof_id].inverse()) + S[dof_id] * qdot(dof_id);
            a_i = a_i.tr(Tr[dof_id].inverse()) + v_i.cross(S[dof_id] * qdot(dof_id));
        }
        
        v[body_id] = v_i;
        a[body_id] = a_i;
        
        if(body_id >= 2)
            break;
        
        v_i = v_i.tr(T[body_id].inverse());
        a_i = a_i.tr(T[body_id].inverse());
        
        body_id++;
    }
    
    dof_id = 5;
    body_id = 2;
    
    /*** Force calculation ***/
    SpForce f_i = SpForce::Zero();
    SpForce f_iB;
    
    for(body_id = 2; body_id >= 0; body_id--) {
        f_i = f_i.tr(T[body_id]);
        f_i += I[body_id] * a[body_id] + v[body_id].cross_star(I[body_id] * v[body_id]);
        
        for(; dof_id > dof_max[body_id]; dof_id--) {
            C(dof_id) = S[dof_id].dot(f_i);
            f_i = f_i.tr(Tr[dof_id]);
        }
    }
    
    return C;
}

Eigen::MatrixXd CRB(const Eigen::VectorXd& q)
{
    unsigned int side = RIGHT;
    
    SpMot S[6] = {
        SpMot(Eigen::Vector3d(0., 0., 1.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(0., 1., 0.), Eigen::Vector3d(0., 0., 0.)),
        SpMot(Eigen::Vector3d(1., 0., 0.), Eigen::Vector3d(0., 0., 0.))
    };
    
    SE3Tr Tr[6] = {
        SE3Tr::RotZ(-q(0)),
        SE3Tr::RotY(-q(1)),
        SE3Tr::RotX(-q(2)),
        SE3Tr::RotY(-q(3)),
        SE3Tr::RotY(-q(4)),
        SE3Tr::RotX(-q(5))
    };
    
    SE3Tr T = SE3Tr::Trans(0., 0., CharacterConst::thighSizeZ);
    SE3Tr T2 = SE3Tr::Trans(0., 0., CharacterConst::shankSizeZ);
    
    Tr[3] = Tr[3] * T;
    Tr[4] = Tr[4] * T2;
    
    Eigen::MatrixXd M(6, 6);
    M.setZero();
    
    SpInertia I[3] = {
        SpInertia(rbMass(B_R_THIGH),
                  Eigen::Vector3d(0., 0., -CharacterConst::thighSizeZ/2.),
                  rbMOI(B_R_THIGH).asDiagonal()),
        SpInertia(rbMass(B_R_SHANK),
                  Eigen::Vector3d(0., 0., -CharacterConst::shankSizeZ/2.),
                  rbMOI(B_R_SHANK).asDiagonal()),
        SpInertia(rbMass(B_R_FOOT),
                  Eigen::Vector3d(CharacterConst::footPosX, 0., -CharacterConst::footSizeZ/2.),
                  rbMOI(B_R_FOOT).asDiagonal())
    };
    
    SpInertia I_comb = I[2];
    
    SE3Tr Tj = SE3Tr::Identity();
    SpForce F;
    
    int have_new_body[6] = { -1, -1, -1, 0, 1, -1 };
    int n_dof = 6;
    
    for(int i=n_dof-1; i>=0; i--) {
        Tj = SE3Tr::Identity();
        F = I_comb * S[i];
        
        for(int j=i; j>=0; j--) {
            M(i,j) = S[j].tr(Tj).dot(F);
            Tj = Tj * Tr[j];
        }
        
        I_comb = I_comb.tr(Tr[i].inverse());
        
        if(have_new_body[i] >= 0)
            I_comb = I_comb + I[have_new_body[i]];
    }
    
    /* Fill up (redundant) upper triangle */
    for(unsigned int i=0; i<6; i++) {
        for(unsigned int j=0; j<i; j++) {
            M(j,i) = M(i,j);
        }
    }
    return M;
}

