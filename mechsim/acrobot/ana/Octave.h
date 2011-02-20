#ifndef _OCTAVE_H_
#define _OCTAVE_H_

#include <octave/oct.h>
#include <ginac/ginac.h>

namespace Octave {

Matrix are(const Matrix& a, const Matrix& b, const Matrix& c);
Matrix matTo(const GiNaC::matrix& mat);
GiNaC::matrix matFrom(const Matrix& mat);

} // end namespace Octave

#endif
