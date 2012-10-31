#include "AcroDyn.h"
#include <iostream>

/* NOTE: these functions are rather inefficient (which is OK, as they are
   intended for testing only). */

Eigen::Matrix<Number, 2, 2> dqddot_dq(const Vector2N& q, const Vector2N& qdot, Number u)
{
    AcroDyn::DResult dresult = AcroDyn::qddot_full(q, qdot, u);
    
    Eigen::Matrix<Number, 2, 2> result;
    result << dresult.df[0], dresult.df[1];
    return result;
}

Eigen::Matrix<Number, 2, 2> dqddot_dqdot(const Vector2N& q, const Vector2N& qdot, Number u)
{
    AcroDyn::DResult dresult = AcroDyn::qddot_full(q, qdot, u);
    
    Eigen::Matrix<Number, 2, 2> result;
    result << dresult.df[2], dresult.df[3];
    return result;
}

Vector2N dqddot_du(const Vector2N& q, const Vector2N& qdot, Number u)
{
    return AcroDyn::qddot_full(q, qdot, u).df[4];
}

Vector2N qddot_a(const Eigen::Matrix<Number, 5, 1>& arg)
{
    // return phiddot(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    AcroDyn::DResult result = AcroDyn::qddot_full(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    return result.f;
}

Eigen::Matrix<Number, 2, 5> dqddot_a(const Eigen::Matrix<Number, 5, 1>& arg)
{
    // return dphiddot(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    AcroDyn::DResult dresult = AcroDyn::qddot_full(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 2, 5> result;
    result << dresult.df[0], dresult.df[1], dresult.df[2], dresult.df[3], dresult.df[4];
    
    return result;
}

Eigen::Matrix<Number, 5, 5> ddqddot_a_1(const Eigen::Matrix<Number, 5, 1>& arg)
{
    AcroDyn::DResult dresult = AcroDyn::qddot_full(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 5, 5> result;
    for(unsigned int a=0; a<5; a++) {
        for(unsigned int b=0; b<5; b++) {
            result(a,b) = dresult.ddf[a][b](0);
        }
    }
    
    return result;
}

Eigen::Matrix<Number, 5, 5> ddqddot_a_2(const Eigen::Matrix<Number, 5, 1>& arg)
{
    AcroDyn::DResult dresult = AcroDyn::qddot_full(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 5, 5> result;
    for(unsigned int a=0; a<5; a++) {
        for(unsigned int b=0; b<5; b++) {
            result(a,b) = dresult.ddf[a][b](1);
        }
    }
    
    return result;
}

void dqddot_test()
{
    Vector2N q0;
    q0 << 1.4, 0.5;
    Vector2N qdot0;
    qdot0 << .3, .9;
    Number u0;
    u0 = 1.1;
    
    Eigen::Matrix<Number, 5, 1> x0;
    x0 << q0, qdot0, u0;
    
    std::cout << qddot_a(x0).transpose() << std::endl;
    std::cout << -4.58464 << " " << 38.6513 << std::endl;
    std::cout << std::endl;
    
    {
        const double h = 1e-6;
        Eigen::Matrix<Number, 2, 5> numeric;
        for(unsigned int i=0; i<5; i++) {
            Eigen::Matrix<Number,5,1> he = h * Eigen::Matrix<Number,5,1>::Unit(5,i);
            numeric.block<2,1>(0,i) = (qddot_a(x0 + he) - qddot_a(x0 - he))/(2*h);
        }
        
        Eigen::Matrix<Number, 2, 5> analytic = dqddot_a(x0);
        
        std::cout << numeric << std::endl;
        std::cout << analytic << std::endl;
        std::cout << "==> " << (numeric - analytic).cwiseAbs().maxCoeff() << std::endl;
    }
    
    {
        const double h = 1e-4;
        Eigen::Matrix<Number, 5, 5> numeric1;
        Eigen::Matrix<Number, 5, 5> numeric2;
        for(unsigned int i=0; i<5; i++) {
            Eigen::Matrix<Number,5,1> he = h * Eigen::Matrix<Number,5,1>::Unit(5,i);
            numeric1.block<1,5>(i,0)
                = (dqddot_a(x0 + he) - dqddot_a(x0 - he)).block<1,5>(0,0)/(2*h);
            numeric2.block<1,5>(i,0)
                = (dqddot_a(x0 + he) - dqddot_a(x0 - he)).block<1,5>(1,0)/(2*h);
        }
        
        std::cout << std::endl;
        std::cout << numeric1 << "\n" << std::endl;
        std::cout << ddqddot_a_1(x0) << "\n" << std::endl;
        std::cout << numeric2 << "\n" << std::endl;
        std::cout << ddqddot_a_2(x0) << std::endl;
        
        std::cout << "==> " << (numeric1 - ddqddot_a_1(x0)).cwiseAbs().maxCoeff() << std::endl;
        std::cout << "==> " << (numeric2 - ddqddot_a_2(x0)).cwiseAbs().maxCoeff() << std::endl;
    }
}

int main()
{
    dqddot_test();
    
    return 0;
}
