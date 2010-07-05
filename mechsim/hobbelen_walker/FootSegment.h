#ifndef __FOOTSEGMENT_H__
#define __FOOTSEGMENT_H__

#include "Vector.h"
#include "Rotation.h"
#include "BodyConf.h"

class FootSegment: public BodyConf
{
  public:
    FootSegment(double m, double I, double l, double w, double r, double h)
        : BodyConf(m, I), fl(l), fw(w), fr(r), fh(h),
          fPos(Vector3::Null), fTheta(0.)
           { }
    
    double l() const { return fl; }
    double w() const { return fw; }
    double r() const { return fr; }
    double h() const { return fh; }
    
    virtual Vector3 CoG() const  { return fPos; }
    virtual Rotation Rot() const { return Rotation(fTheta, Vector3::eY); }
    
    Vector3 p1() const;   // Attach point
    Vector3 pf() const;   // Front contact circle center
    Vector3 pb() const;   // Back contact circle center
    Vector3 pfb() const;  // Front contact circle ctr in body (unrotated) coordinates
    Vector3 pbb() const;  // Back contact circle ctr in body (unrotated) coordinates
    
    void SetCoG(Vector3 pos, double theta);
    void SetP1(Vector3 p1, double theta);
    void SetPF(Vector3 pf, double theta);
    void SetPB(Vector3 pb, double theta);
    
  private:
    double fl, fw, fr, fh;
    Vector3 fPos;
    double fTheta;
    
    inline Vector3 rbfhat() const
        { return Vector3(cos(fTheta), 0., -sin(fTheta)); }
    inline Vector3 nor() const
        { return Vector3(sin(fTheta), 0.,  cos(fTheta)); }
    
};

#endif
