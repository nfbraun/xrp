#include "AcroDyn.h"

namespace SystemConst {
    const double G = 9.81;
    const double M1 = .5;
    const double M2 = .2;
    const double LC = .3;
    const double L1 = .5;
    const double L2 = .6;
    const double I1 = 0.;
    const double I2 = 0.;
    // const double GAMMA = 0.;
}

AcroDyn::DResult AcroDyn::qddot_full(const Vector_Ndof& q, const Vector_Ndof& qdot,
                                     const Vector_Nctrl& u)
{
    using namespace SystemConst;
    
    // Note that the mass matrix only depends on q[1], not on q[0]
    Eigen::Matrix<Number, 2, 2> m;
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    // First derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> dm_dq[2];
    dm_dq[0].setZero(2,2);
    
    dm_dq[1](0,0) = -2.*M2*L1*L2*sin(q[1]);
    dm_dq[1](0,1) = -M2*L1*L2*sin(q[1]);
    dm_dq[1](1,0) = dm_dq[1](0,1);
    dm_dq[1](1,1) = 0.;
    
    // Second derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> d2m_dqq[2][2];
    d2m_dqq[0][0].setZero(2,2);
    d2m_dqq[0][1].setZero(2,2);
    d2m_dqq[1][0].setZero(2,2);
    
    d2m_dqq[1][1](0,0) = -2.*M2*L1*L2*cos(q[1]);
    d2m_dqq[1][1](0,1) = -M2*L1*L2*cos(q[1]);
    d2m_dqq[1][1](1,0) = d2m_dqq[1][1](0,1);
    d2m_dqq[1][1](1,1) = 0.;
    
    // Third derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> d3m_dqqq[2][2][2];
    d3m_dqqq[0][0][0].setZero(2,2);
    d3m_dqqq[0][0][1].setZero(2,2);
    d3m_dqqq[0][1][0].setZero(2,2);
    d3m_dqqq[0][1][1].setZero(2,2);
    d3m_dqqq[1][0][0].setZero(2,2);
    d3m_dqqq[1][0][1].setZero(2,2);
    d3m_dqqq[1][1][0].setZero(2,2);
    
    d3m_dqqq[1][1][1](0,0) = 2.*M2*L1*L2*sin(q[1]);
    d3m_dqqq[1][1][1](0,1) = M2*L1*L2*sin(q[1]);
    d3m_dqqq[1][1][1](1,0) = d3m_dqqq[1][1][1](0,1);
    d3m_dqqq[1][1][1](1,1) = 0.;
    
    // Potential energy
    Eigen::Matrix<Number, 2, 1> dV_dq;
    dV_dq(0) = -(M1*LC + M2*L1)*G*sin(q[0]) - M2*L2*G*sin(q[0] + q[1]);
    dV_dq(1) = -M2*L2*G*sin(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d2V_dqq[2];
    d2V_dqq[0](0) = -(M1*LC + M2*L1)*G*cos(q[0]) - M2*L2*G*cos(q[0] + q[1]);
    d2V_dqq[0](1) = -M2*L2*G*cos(q[0] + q[1]);
    
    d2V_dqq[1](0) = d2V_dqq[0](1);
    d2V_dqq[1](1) = -M2*L2*G*cos(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d3V_dqqq[2][2];
    d3V_dqqq[0][0](0) = (M1*LC + M2*L1)*G*sin(q[0]) + M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[0][0](1) = M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[0][1](1) = M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[1][1](1) = M2*L2*G*sin(q[0] + q[1]);
    
    d3V_dqqq[0][1](0) = d3V_dqqq[0][0](1);
    d3V_dqqq[1][0](0) = d3V_dqqq[0][0](1);
    d3V_dqqq[1][0](1) = d3V_dqqq[0][1](1);
    d3V_dqqq[1][1](0) = d3V_dqqq[0][1](1);
    
    // Inverse mass matrix and its derivatives
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 2> dmi_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dmi_dq[a] = -mi * dm_dq[a] * mi;
    }
    
    Eigen::Matrix<Number, 2, 2> d2mi_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2mi_dqq[a][b] = -dmi_dq[a]*dm_dq[b]*mi - dmi_dq[b]*dm_dq[a]*mi - mi*d2m_dqq[a][b]*mi;
        }
    }
    
    // Coriolis term c(q, qdot)
    Eigen::Matrix<Number, 2, 1> c;
    c.setZero(2, 1);
    for(unsigned int l=0; l<2; l++) {
        for(unsigned int i=0; i<2; i++) {
            for(unsigned int j=0; j<2; j++) {
                c(l) += (dm_dq[j](i,l) - dm_dq[l](i,j)/2.)*qdot[i]*qdot[j];
            }
        }
    }
    
    // ... and its first derivatives
    Eigen::Matrix<Number, 2, 1> dc_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dc_dq[a].setZero(2,1);
        for(unsigned int l=0; l<2; l++) {
            for(unsigned int i=0; i<2; i++) {
                for(unsigned int j=0; j<2; j++) {
                    dc_dq[a](l) += (d2m_dqq[a][j](i,l) - d2m_dqq[a][l](i,j)/2.)*qdot[i]*qdot[j];
                }
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot[2];
    for(unsigned int a=0; a<2; a++) {
        dc_dqdot[a].setZero(2,1);
        for(unsigned int l=0; l<2; l++) {
            for(unsigned int j=0; j<2; j++) {
                dc_dqdot[a](l) += (dm_dq[j](a,l) - dm_dq[l](a,j)/2.) * qdot[j];
            }
            for(unsigned int i=0; i<2; i++) {
                dc_dqdot[a](l) += (dm_dq[a](i,l) - dm_dq[l](i,a)/2.) * qdot[i];
            }
        }
    }
    
    // ... and its second derivatives
    Eigen::Matrix<Number, 2, 1> d2c_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2c_dqq[a][b].setZero(2,1);
            for(unsigned int l=0; l<2; l++) {
                for(unsigned int i=0; i<2; i++) {
                    for(unsigned int j=0; j<2; j++) {
                        d2c_dqq[a][b](l) +=
                            (d3m_dqqq[a][b][j](i,l) - d3m_dqqq[a][b][l](i,j)/2.)*qdot[i]*qdot[j];
                    }
                }
            }
        }
    }
    
    // Assumes m(a,b) = m(b,a)
    Eigen::Matrix<Number, 2, 1> d2c_dqdotqdot[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            for(unsigned int l=0; l<2; l++) {
                d2c_dqdotqdot[a][b](l) = (dm_dq[b](a,l) + dm_dq[a](b,l) - dm_dq[l](a,b));
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> d2c_dq_dqdot[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2c_dq_dqdot[a][b].setZero(2,1);
            for(unsigned int l=0; l<2; l++) {
                for(unsigned int j=0; j<2; j++) {
                    d2c_dq_dqdot[a][b](l) += (d2m_dqq[a][j](b,l) - d2m_dqq[a][l](b,j)/2.) * qdot[j];
                }
                for(unsigned int i=0; i<2; i++) {
                    d2c_dq_dqdot[a][b](l) += (d2m_dqq[a][b](i,l) - d2m_dqq[a][l](i,b)/2.) * qdot[i];
                }
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, u;
    
    // Helper g and derivatives
    Eigen::Matrix<Number, 2, 1> g = b - c - dV_dq;
    
    Eigen::Matrix<Number, 2, 1> dg_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dg_dq[a] = -dc_dq[a] - d2V_dqq[a];
    }
    
    Eigen::Matrix<Number, 2, 1> d2g_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2g_dqq[a][b] = -d2c_dqq[a][b] - d3V_dqqq[a][b];
        }
    }
    
    DResult result;
    
    // f
    result.f = mi * (b - c - dV_dq);
    
    // df_dq
    result.df[0] = dmi_dq[0] * g + mi * dg_dq[0];
    result.df[1] = dmi_dq[1] * g + mi * dg_dq[1];
    
    // df_qdot
    result.df[2] = mi * (-dc_dqdot[0]);
    result.df[3] = mi * (-dc_dqdot[1]);
    
    // df_du
    Eigen::Matrix<Number, 2, 1> bp;
    bp << 0, 1;
    
    result.df[4] = mi * bp;
    
    // d2f_dqq
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddf[a][b] = d2mi_dqq[a][b] * g + dmi_dq[a] * dg_dq[b] 
                                  + dmi_dq[b] * dg_dq[a] + mi * d2g_dqq[a][b];
        }
    }
    
    // d2f_dqdotqdot
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddf[a+2][b+2] = -mi * d2c_dqdotqdot[a][b];
        }
    }
    
    // d2f_duu
    result.ddf[4][4] << 0., 0.;
    
    // d2f_dq_dqdot
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddf[a][b+2] = -dmi_dq[a] * dc_dqdot[b] - mi * d2c_dq_dqdot[a][b];
            result.ddf[b+2][a] = result.ddf[a][b+2];
        }
    }
    
    // d2f_dq_du
    result.ddf[0][4] = dmi_dq[0] * bp;
    result.ddf[1][4] = dmi_dq[1] * bp;
    
    result.ddf[4][0] = result.ddf[0][4];
    result.ddf[4][1] = result.ddf[1][4];
    
    // d2f_dqdot_du
    result.ddf[2][4] << 0., 0.;
    result.ddf[3][4] << 0., 0.;
    
    result.ddf[4][2] = result.ddf[2][4];
    result.ddf[4][3] = result.ddf[3][4];
    
    return result;
}
