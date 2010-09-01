#include <vector>
#include <iostream>
#include <ginac/ginac.h>
#include "GiJIT.h"

int main()
{
    GiNaC::symbol x("x"), y("y"), z("z");
    //GiNaC::ex ex = sqrt(pow(x,2) + pow(y,2) + pow(z, 2));
    GiNaC::matrix mat(1, 3);
    mat = x, y, z;
    
    GiNaC::ex ex = sqrt((mat * mat.transpose()).evalm());
    // std::cout << ex.diff(x)*ex << std::endl;
    
    typedef GiJIT::CodeGenV<GiJIT::Result, GiJIT::Vector> codegen;
    
    codegen::func_t func = codegen::compile(
        GiNaC::lst(ex.diff(x)*ex, ex.diff(y)*ex, ex.diff(z)*ex),
        GiNaC::lst(x,y,z));
    
    GiJIT::dump();
    
    double vx[] = { 3., 4., 12. };
    double result[3];
    
    func(result, vx);
    
    // std::cout << func(vx) << std::endl;
    
    //func(vx, vx[2], &result);
    
    std::cout << result[0] << " " << result[1] << " " << result[2] << std::endl;
    
    return 0;
}
