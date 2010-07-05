#ifndef __BODYCONF_H__
#define __BODYCONF_H__

#include "Vector.h"
#include "Rotation.h"

class BodyConf
{
  public:
    BodyConf(double m, double I)
        : fm(m), fI(I) { }
    
    double m() const { return fm; }
    double I() const { return fI; }
    
    virtual Vector3 CoG() const = 0;
    virtual Rotation Rot() const = 0;
  
  protected:
    double fm, fI;
};

#endif
