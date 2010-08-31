#ifndef _ODEGENERATOR_H_
#define _ODEGENERATOR_H_

#include <vector>
#include <string>
#include <ginac/ginac.h>

class ODEGenerator
{
  public:
    ODEGenerator(std::string sysname, int ndim, GiNaC::ex qdotdot,
                 std::vector<GiNaC::symbol> qdot,
                 std::vector<GiNaC::symbol> q,
                 bool cxxmode = true);
    void Generate(std::ostream& s);
    
  private:
    std::string fName;
    int fNdim;
    GiNaC::ex fQdotdot;
    std::vector<GiNaC::symbol> fQdot, fQ;
    bool fCXXMode;
};

#endif
