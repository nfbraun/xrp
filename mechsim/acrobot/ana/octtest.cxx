// Solution according to MATLAB documentation:
//  0.5895    1.8216
//  1.8216    8.8188

#include <iostream>
#include "Octave.h"

int main()
{
    Matrix a(2, 2);
    a(0, 0) = -3;
    a(0, 1) =  2;
    a(1, 0) =  1;
    a(1, 1) =  1;
    
    Matrix b(2, 1);
    b(0, 0) = 0;
    b(1, 0) = 1;
    
    Matrix c(1, 2);
    c(0, 0) =  1;
    c(0, 1) = -1;
    
    double r = 3;
    
    Matrix x = Octave::are(a, b * b.transpose() / r, c.transpose() * c);
    
    std::cout << x;
    
    return 0;
}
