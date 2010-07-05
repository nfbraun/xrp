#ifndef __CHAINSEGMENT_H__
#define __CHAINSEGMENT_H__

#include "Vector.h"
#include "Rotation.h"
#include "BodyConf.h"

class ChainSegment: public BodyConf {
  public:
    ChainSegment(double m, double I, double l, double c, double w)
      : BodyConf(m, I), fl(l), fc(c), fw(w), fPos(Vector3::Null), fTheta(0.) { }
    
    double l() const { return fl; }
    double c() const { return fc; }
    double w() const { return fw; }
    
    virtual Vector3 CoG() const  { return fPos; }
    virtual Rotation Rot() const { return Rotation(fTheta, Vector3::eY); }
    
    Vector3 p1() const;
    Vector3 p2() const;
    Vector3 cb() const;   // Chain segment ctr in body (unrotated) coordinates
    
    void SetCoG(Vector3 pos, double theta);
    void SetP1(Vector3 p1, double theta);
    void SetP2(Vector3 p2, double theta);
    void SetP1P2(Vector3 p1, Vector3 p2);
    
  private:
    double fl, fc, fw;
    Vector3 fPos;
    double fTheta;
    
    inline Vector3 r12hat() const
        { return Vector3(-sin(fTheta), 0., -cos(fTheta)); }
    inline Vector3 nor() const
        { return Vector3( cos(fTheta), 0., -sin(fTheta)); }
};

#endif
