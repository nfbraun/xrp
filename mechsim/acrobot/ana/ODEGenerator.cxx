#include "ODEGenerator.h"

ODEGenerator::ODEGenerator(std::string sysname, int ndim, GiNaC::ex qdotdot,
                 std::vector<GiNaC::symbol> qdot,
                 std::vector<GiNaC::symbol> q,
                 bool cxxmode)
    : fName(sysname), fNdim(ndim), fQdotdot(qdotdot), fQdot(qdot), fQ(q),
      fCXXMode(cxxmode)
{ }

void ODEGenerator::Generate(std::ostream& s)
{
    using std::endl;

    s << "// This file is generated, DO NOT EDIT!" << endl;
    if(fCXXMode)
        s << "#include <cmath>" << endl;
    else
        s << "#include <math.h>" << endl;
    s << "#include <gsl/gsl_errno.h>" << endl;
    s << endl;
    
    s << "int " << fName << "_deriv(double t, const double y[], double f[], void *_p)" << endl;
    s << "{" << endl;
    for(int i=0; i<fNdim; i++)
        s << "    const double qdot" << i << " = y[" << i << "];" << endl;
    for(int i=0; i<fNdim; i++)
        s << "    const double q" << i << " = y[" << (i+fNdim) << "];" << endl;
    
    s << endl;
    s << GiNaC::csrc_double;
    
    for(int i=0; i<fNdim; i++)
        s << "    f[" << i << "] = " << fQdotdot[i] << ";" << endl;
    
    for(int i=0; i<fNdim; i++)
        s << "    f[" << (i+fNdim) << "] = qdot" << i << ";" << endl;
    
    s << endl;
    s << "    return GSL_SUCCESS;" << endl;
    s << "}" << endl;
}
