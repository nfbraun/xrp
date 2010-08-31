#include <cmath>
#include <iostream>
#include "Lagrange.h"
#include "ODEGenerator.h"
#include "Acrobot.h"

int main()
{
    Acrobot a;
    Lagrange l(a);
    
    ODEGenerator g("acrobot", 2, l.qdotdot(), l.qdot(), l.q());
    g.Generate(std::cout);
    
    return 0;
}
