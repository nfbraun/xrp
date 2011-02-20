#ifndef _LQR_H_
#define _LQR_H_

#include "Lagrange.h"
#include <ginac/ginac.h>

class LQR
{
  public:
    static GiNaC::matrix Sys_A(const Lagrange& l, const GiNaC::symbol& u1);
    static GiNaC::matrix Sys_B(const Lagrange& l, const GiNaC::symbol& u1);
    static GiNaC::matrix lqrControl(const Lagrange& l, const GiNaC::symbol& u1);
};

#endif
