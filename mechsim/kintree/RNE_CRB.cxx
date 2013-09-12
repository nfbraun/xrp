#include "RNE_CRB.h"
#include <Eigen/Dense>
#include "Spatial.h"

SpForce calc_react(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g)
{
    SpMot v[sys.nJoints()];
    SpMot a[sys.nJoints()];
    
    SpMot v_i = SpMot::Zero();
    SpMot a_i = SpMot(Eigen::Vector3d::Zero(), -g);
    
    int body_id = 0;
    int dof_id = 0;
    
    /*** Velocity/acceleration calculation ***/
    while(1) {
        for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id++) {
            v_i = v_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qdot(dof_id);
            a_i = a_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qddot(dof_id)
                + v_i.cross(sys.S(dof_id) * qdot(dof_id));
        }
        
        v[body_id] = v_i;
        a[body_id] = a_i;
        
        if(body_id >= (sys.nJoints() - 1))
            break;
        
        v_i = v_i.tr(sys.bodyT(body_id).inverse());
        a_i = a_i.tr(sys.bodyT(body_id).inverse());
        
        body_id++;
    }
    
    dof_id--;
    assert(dof_id == (sys.nDof() - 1));
    assert(body_id == (sys.nJoints() - 1));
    
    /*** Force calculation ***/
    SpForce f_i = SpForce::Zero();
    
    for(body_id; body_id >= 0; body_id--) {
        f_i = f_i.tr(sys.bodyT(body_id));
        f_i += sys.I(body_id) * a[body_id] + v[body_id].cross_star(sys.I(body_id) * v[body_id]);
        
        for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id--) {
            f_i = f_i.tr(expSq(dof_id));
        }
    }
    
    return f_i;
}

Eigen::VectorXd calc_u(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g)
{
    Eigen::VectorXd C(sys.nDof());
    C.setZero();
    
    SpMot v[sys.nJoints()];
    SpMot a[sys.nJoints()];
    
    SpMot v_i = SpMot::Zero();
    SpMot a_i = SpMot(Eigen::Vector3d::Zero(), -g);
    
    int body_id = 0;
    int dof_id = 0;
    
    /*** Velocity/acceleration calculation ***/
    while(1) {
        for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id++) {
            v_i = v_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qdot(dof_id);
            a_i = a_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qddot(dof_id)
                + v_i.cross(sys.S(dof_id) * qdot(dof_id));
        }
        
        v[body_id] = v_i;
        a[body_id] = a_i;
        
        if(body_id >= (sys.nJoints() - 1))
            break;
        
        v_i = v_i.tr(sys.bodyT(body_id).inverse());
        a_i = a_i.tr(sys.bodyT(body_id).inverse());
        
        body_id++;
    }
    
    dof_id--;
    assert(dof_id == (sys.nDof() - 1));
    assert(body_id == (sys.nJoints() - 1));
    
    /*** Force calculation ***/
    SpForce f_i = SpForce::Zero();
    
    for(body_id; body_id >= 0; body_id--) {
        f_i = f_i.tr(sys.bodyT(body_id));
        f_i += sys.I(body_id) * a[body_id] + v[body_id].cross_star(sys.I(body_id) * v[body_id]);
        
        for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id--) {
            C(dof_id) = sys.S(dof_id).dot(f_i);
            f_i = f_i.tr(expSq(dof_id));
        }
    }
    
    return C;
}

Eigen::MatrixXd calc_du_dq(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot, const Eigen::Vector3d& g)
{
    SpMot v[sys.nJoints()];
    SpMot a[sys.nJoints()];
    SpMot d_v[sys.nJoints()];
    SpMot d_a[sys.nJoints()];
    
    Eigen::MatrixXd du_dq(sys.nDof(), sys.nDof());
    
    for(unsigned int diff_id=0; diff_id<sys.nDof(); diff_id++) {
        SpMot a_0 = SpMot(Eigen::Vector3d::Zero(), -g);
        
        SpMot v_i = SpMot::Zero();
        SpMot a_i = a_0;
        SpMot d_v_i = SpMot::Zero();
        SpMot d_a_i = SpMot::Zero();
        
        int body_id = 0;
        int dof_id = 0;
        
        /*** Velocity/acceleration calculation ***/
        while(1) {
            for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id++) {
                v_i = v_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qdot(dof_id);
                a_i = a_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qddot(dof_id)
                      + v_i.cross(sys.S(dof_id) * qdot(dof_id));
                a_0 = a_0.tr(expSq(dof_id).inverse());
                
                d_v_i = d_v_i.tr(expSq(dof_id).inverse());
                if(dof_id == diff_id)
                    d_v_i = -sys.S(diff_id).cross(d_v_i);
                if(dof_id < diff_id)
                    d_v_i += sys.S(dof_id) * qdot(dof_id);
                
                d_a_i = d_a_i.tr(expSq(dof_id).inverse());
                if(dof_id == diff_id)
                    d_a_i = -sys.S(diff_id).cross(d_a_i) - sys.S(diff_id).cross(a_0);
                if(dof_id < diff_id) {
                    d_a_i += sys.S(dof_id) * qddot(dof_id);
                }
                d_a_i += d_v_i.cross(sys.S(dof_id) * qdot(dof_id));
            }
            
            v[body_id] = v_i;
            a[body_id] = a_i;
            if(dof_id <= diff_id) {
                d_v[body_id] = SpMot::Zero();
                d_a[body_id] = SpMot::Zero();
            } else {
                d_v[body_id] = d_v_i;
                d_a[body_id] = d_a_i;
            }
            
            if(body_id >= (sys.nJoints() - 1))
                break;
            
            v_i = v_i.tr(sys.bodyT(body_id).inverse());
            a_i = a_i.tr(sys.bodyT(body_id).inverse());
            d_v_i = d_v_i.tr(sys.bodyT(body_id).inverse());
            d_a_i = d_a_i.tr(sys.bodyT(body_id).inverse());
            
            body_id++;
        }
        
        dof_id--;
        assert(dof_id == (sys.nDof() - 1));
        assert(body_id == (sys.nJoints() - 1));
        
        /*** Force calculation ***/
        SpForce f_i = SpForce::Zero();
        SpForce d_f_i = SpForce::Zero();
        
        for(body_id; body_id >= 0; body_id--) {
            f_i = f_i.tr(sys.bodyT(body_id));
            f_i += sys.I(body_id) * a[body_id] + v[body_id].cross_star(sys.I(body_id) * v[body_id]);
            
            d_f_i = d_f_i.tr(sys.bodyT(body_id));
            d_f_i += sys.I(body_id) * d_a[body_id] + d_v[body_id].cross_star(sys.I(body_id) * v[body_id])
                + v[body_id].cross_star(sys.I(body_id) * d_v[body_id]);
            
            for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id--) {
                du_dq(dof_id, diff_id) = sys.S(dof_id).dot(d_f_i);
                
                f_i = f_i.tr(expSq(dof_id));
                d_f_i = d_f_i.tr(expSq(dof_id));
                
                if(dof_id == diff_id)
                    d_f_i += sys.S(diff_id).cross_star(f_i);
            }
        }
    }
    
    return du_dq;
}

Eigen::MatrixXd calc_du_dqdot(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq, const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot)
{
    SpMot v[sys.nJoints()];
    SpMot d_v[sys.nJoints()];
    SpMot d_a[sys.nJoints()];
    
    Eigen::MatrixXd du_dqdot(sys.nDof(), sys.nDof());
    
    for(unsigned int diff_id=0; diff_id<sys.nDof(); diff_id++) {
        SpMot v_i = SpMot::Zero();
        SpMot d_v_i = SpMot::Zero();
        SpMot d_a_i = SpMot::Zero();
        
        int body_id = 0;
        int dof_id = 0;
        
        /*** Velocity/acceleration calculation ***/
        while(1) {
            for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id++) {
                v_i = v_i.tr(expSq(dof_id).inverse()) + sys.S(dof_id) * qdot(dof_id);
                
                d_v_i = d_v_i.tr(expSq(dof_id).inverse());
                if(dof_id == diff_id)
                    d_v_i = sys.S(diff_id);
                
                d_a_i = d_a_i.tr(expSq(dof_id).inverse());
                if(dof_id == diff_id)
                    d_a_i = v_i.cross(sys.S(diff_id));
                
                d_a_i += d_v_i.cross(sys.S(dof_id) * qdot(dof_id));
            }
            
            v[body_id] = v_i;
            if(dof_id <= diff_id) {
                d_v[body_id] = SpMot::Zero();
                d_a[body_id] = SpMot::Zero();
            } else {
                d_v[body_id] = d_v_i;
                d_a[body_id] = d_a_i;
            }
            
            if(body_id >= (sys.nJoints() - 1))
                break;
            
            v_i = v_i.tr(sys.bodyT(body_id).inverse());
            d_v_i = d_v_i.tr(sys.bodyT(body_id).inverse());
            d_a_i = d_a_i.tr(sys.bodyT(body_id).inverse());
            
            body_id++;
        }
        
        dof_id--;
        assert(dof_id == (sys.nDof() - 1));
        assert(body_id == (sys.nJoints() - 1));
        
        /*** Force calculation ***/
        SpForce d_f_i = SpForce::Zero();
        
        for(body_id; body_id >= 0; body_id--) {
            d_f_i = d_f_i.tr(sys.bodyT(body_id));
            d_f_i += sys.I(body_id) * d_a[body_id] + d_v[body_id].cross_star(sys.I(body_id) * v[body_id])
                + v[body_id].cross_star(sys.I(body_id) * d_v[body_id]);
            
            for(int jnt_dof = 0; jnt_dof < sys.nJntDof(body_id); jnt_dof++, dof_id--) {
                du_dqdot(dof_id, diff_id) = sys.S(dof_id).dot(d_f_i);
                
                d_f_i = d_f_i.tr(expSq(dof_id));
            }
        }
    }
    
    return du_dqdot;
}

Eigen::MatrixXd calc_M(const KinChain2& sys, const Eigen::Matrix<SE3Tr, Eigen::Dynamic, 1>& expSq)
{
    Eigen::MatrixXd M(sys.nDof(), sys.nDof());
    M.setZero();
    
    SpInertia I_comb = sys.I(sys.nJoints()-1);
    
    SE3Tr Tj = SE3Tr::Identity();
    SpForce F;
    
    SE3Tr Tr[sys.nDof()];
    int dof_id = sys.nDof();
    for(int joint_id=sys.nJoints()-1; joint_id>=0; joint_id--) {
        for(int jnt_dof=0; jnt_dof < sys.nJntDof(joint_id); jnt_dof++) {
            dof_id--;
            Tr[dof_id] = expSq(dof_id).inverse();
        }
        if(joint_id > 0)
            Tr[dof_id] = Tr[dof_id] * sys.bodyT(joint_id-1).inverse();
    }
    
    int dof_i = sys.nDof();
    for(int body_id=sys.nJoints()-1; body_id>=0; body_id--) {
        for(int jnt_dof=0; jnt_dof < sys.nJntDof(body_id); jnt_dof++) {
            dof_i--;
            
            Tj = SE3Tr::Identity();
            F = I_comb * sys.S(dof_i);
            
            for(int dof_j=dof_i; dof_j>=0; dof_j--) {
                M(dof_i, dof_j) = sys.S(dof_j).tr(Tj).dot(F);
                Tj = Tj * Tr[dof_j];
            }
            
            I_comb = I_comb.tr(Tr[dof_i].inverse());
        }
        if(body_id > 0)
            I_comb = I_comb + sys.I(body_id-1);
    }
    
    /* Fill up (redundant) upper triangle */
    for(unsigned int i=0; i<sys.nDof(); i++) {
        for(unsigned int j=0; j<i; j++) {
            M(j,i) = M(i,j);
        }
    }
    return M;
}

