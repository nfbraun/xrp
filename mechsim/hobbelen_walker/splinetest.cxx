#include <iostream>
#include <cmath>
#include "Spline.h"

int main()
{
    const int N_POINTS = 4;
    const double x[N_POINTS] = {  0.,   0.1,   0.7,  1. };
    const double y[N_POINTS] = { -0.52, -0.52, 0.57, 0.52 };
    
    Spline spline(N_POINTS, x, y);
    
    std::cout << "#:1:y_interp" << std::endl;
    for(int i=0; i<=1000; i++) {
        double x = i / 1000.;
        double y_interp = spline.eval(x);
        std::cout << x << " " << y_interp << std::endl;
    }
    
    return 0;
}

/* int main()
{
    const int N_POINTS = 11;
    double x[N_POINTS], y[N_POINTS];
    
    for(int i=0; i<N_POINTS; i++) {
        x[i] = (double)i / (N_POINTS-1);
        y[i] = sin(x[i]*4*M_PI);
    }
    
    Spline spline(N_POINTS, x, y);
    
    std::cout << "#:1:y_real" << std::endl;
    std::cout << "#:2:y_interp" << std::endl;
    for(int i=0; i<=1000; i++) {
        double x = i / 1000.;
        double y_real = sin(x*4*M_PI);
        double y_interp = spline.eval(x);
        std::cout << x << " " << y_real << " " << y_interp << std::endl;
    }
    
    return 0;
} */
