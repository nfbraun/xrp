#include <vector>
#include <iostream>
#include <cmath>
#include <ginac/ginac.h>
#include "GiJIT.h"
#include <stdint.h>

void test1()
{
    std::cout << "Test1:" << std::endl;
    {
        GiNaC::symbol q0("q0"), q1("q1"), qdot0("qdot0"), qdot1("qdot1");
        GiNaC::ex ex = -( -2.*cos(q1)-3.)/( 2.*cos(q1)+3.)/( pow(cos(q1),2.0)-2.)*( -0.1*qdot1-(qdot0*qdot0)*sin(q1)+9.81*sin( q0+q1))*( cos(q1)+1.0)-( 19.62*sin(q0)+9.81*sin(q0+q1)+-0.1*qdot0+2.*qdot1*qdot0*sin(q1)+(qdot1*qdot1)*sin(q1))/( pow(cos(q1),2.0)-2.);
        
        typedef GiJIT::CodeGen<GiJIT::Number, GiJIT::Number, GiJIT::Number, GiJIT::Number> codegen;
        codegen::func_t func = codegen::compile(ex, qdot0, qdot1, q0, q1);
        
        // GiJIT::dump();
        
        std::cout << "  Result:   " << func(.4, .3, 1.5, 2.3) << std::endl;
    }
    
    {
        double q0 = 1.5, q1 = 2.3, qdot0 = .4, qdot1 = .3;
        double ex = -( -2.*cos(q1)-3.)/( 2.*cos(q1)+3.)/( pow(cos(q1),2.0)-2.)*( -0.1*qdot1-(qdot0*qdot0)*sin(q1)+9.81*sin( q0+q1))*( cos(q1)+1.0)-( 19.62*sin(q0)+9.81*sin(q0+q1)+-0.1*qdot0+2.*qdot1*qdot0*sin(q1)+(qdot1*qdot1)*sin(q1))/( pow(cos(q1),2.0)-2.);
        
        std::cout << "  Expected: " << ex << std::endl;
    }
}

void test2()
{
    GiNaC::symbol x("x"), y("y"), z("z");
    GiNaC::matrix mat(1, 3);
    mat = x, y, z;
    
    GiNaC::ex ex = sqrt((mat * mat.transpose()).evalm());
        
    GiNaC::matrix rmat(2, 2);
    rmat = ex.diff(x)*ex, ex.diff(y)*ex,
           ex.diff(z)*ex, 4.*atan(1.);
    
    typedef GiJIT::CodeGenR<GiJIT::ReturnDummyInt,
        GiJIT::DummyArg<void *>, GiJIT::Result, GiJIT::Vector > codegen;
    
    codegen::func_t func = codegen::compile(42, 0,
        rmat, GiNaC::lst(x,y,z));
    
    // GiJIT::dump();
        
    double vx[] = { 3., 4., 12. };
    double result[4];
    
    std::cout << "Test2:" << std::endl;
    std::cout << "  Result:   ";
    std::cout << func(0, result, vx) << " ";
    std::cout << result[0] << " " << result[1] << " " << result[2] << " ";
    std::cout << result[3] << std::endl;
    
    std::cout << "  Expected: ";
    std::cout << 42 << " " << vx[0] << " " << vx[1] << " " << vx[2] << " ";
    std::cout << M_PI << std::endl;
}

void test3()
{
    // Test case for bug found on 2010-09-21.
    // This only needs to run through without an LLVM error.
    
    GiNaC::symbol(x);
    
    GiNaC::ex f = tan(x);
    typedef GiJIT::CodeGen< GiJIT::Number > codegen;
    codegen::func_t cf = codegen::compile(f, x);
    codegen::func_t cf2 = codegen::compile(f, x);
    
    // GiJIT::dump();
    
    std::cout << "Test3: passed" << std::endl;
}

int main()
{
    std::cout.precision(17);
    test1();
    test2();
    test3();
    
    return 0;
}
