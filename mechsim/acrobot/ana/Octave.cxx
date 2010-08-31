#include <iostream>
#include <stdexcept>
#include <octave/oct.h>
#include <octave/octave.h>
#include <octave/parse.h>
#include <octave/toplev.h>

class OctRunner {
  public:
    OctRunner();
    ~OctRunner();
};

OctRunner::OctRunner()
{
    // Silence compiler warning
    // (octave_main() takes char** rather than const char**)
    char arg0[16] = "octave";
    char arg1[16] = "--silent";
    char* argv[] = { arg0, arg1 };
    
    if(!octave_main(2, argv, true)) {
        throw std::runtime_error("Failed to start octave");
    }
}

OctRunner::~OctRunner()
{
    do_octave_atexit();
}

OctRunner _octrunner;


namespace Octave {

Matrix are(const Matrix& a, const Matrix& b, const Matrix& c)
{
    octave_value_list f_arg, f_ret;
    
    f_arg(0) = a;
    f_arg(1) = b;
    f_arg(2) = c;
    
    f_ret = feval("are", f_arg);
    
    return f_ret(0).matrix_value();
}

} // end namespace Octave
