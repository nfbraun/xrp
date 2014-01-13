#ifndef MSIM_SHADOW_H
#define MSIM_SHADOW_H

#include <stdlib.h>
#include <stdarg.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <cassert>
#include <iostream>
#include <vector>
#include <GL/gl.h>
#include <Eigen/Dense>

// plane equation
struct sPlaneEq
{
    Eigen::Vector3f n;
    float d;
};

// structure describing an object's face
class sPlane
{
  public:
    sPlane() { memset( neigh, 0, sizeof neigh ); }
    
    sPlane(unsigned int p0, unsigned int p1, unsigned int p2) {
        memset( neigh, 0, sizeof neigh );
        p[0] = p0; p[1] = p1; p[2] = p2;
        fNormals[0].setZero();
        fNormals[1].setZero();
        fNormals[2].setZero();
    }
    
    void setNormals(const Eigen::Vector3f& n)
    {
        fNormals[0] = n;
        fNormals[1] = n;
        fNormals[2] = n;
    }
    
    void setNormals(const Eigen::Vector3f& n0, const Eigen::Vector3f& n1, const Eigen::Vector3f& n2)
    {
        fNormals[0] = n0;
        fNormals[1] = n1;
        fNormals[2] = n2;
    }
    
    const Eigen::Vector3f& normal(unsigned int j) const
        { assert(j < 3); return fNormals[j]; }
    
  //private:
    unsigned int p[3];
    Eigen::Vector3f fNormals[3];
    unsigned int neigh[3];
    sPlaneEq fPlaneEq;
};

// object structure
class Object
{
  public:
    Object();
    
    // Abuse Eigen::Matrix as alignment-safe storage class
    Eigen::Matrix<Eigen::Vector3f, Eigen::Dynamic, 1> fPoints;
    Eigen::Matrix<sPlane, Eigen::Dynamic, 1> fPlanes;
    
    void setColor(const Eigen::Vector4f& c)
        { fColor = c; }
    void setColor(float r, float g, float b, float a=1.0f)
        { setColor(Eigen::Vector4f(r, g, b, a)); }
    Eigen::Vector4f color() const { return fColor; }
    
    void setCenter(const Eigen::Vector3f& c);
    void setCenter(float x, float y, float z) { setCenter(Eigen::Vector3f(x, y, z)); }
    void setRotation(const Eigen::Quaternionf& q);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    void updateConnectivity();
    virtual void updatePoints() = 0;
    virtual void updateNormals() = 0;
    void updatePlaneEq();
    
  protected:
    Eigen::Vector3f fC;
    Eigen::Quaternionf fQ;
    
    Eigen::Vector4f fColor;
};

class Box: public Object
{
  public:
    Box(float lx, float ly, float lz);
    
    void updatePoints();
    void updateNormals();
    
  private:
    float fsx, fsy, fsz;
};

class Cylinder: public Object
{
  public:
    Cylinder(float r, float l, unsigned int nseg=16);
    
    void updatePoints();
    void updateNormals();
  
  private:
    float fr, fs;
    unsigned int fNSeg;
};

class Scene
{
  public:
    Scene() : fLightPos(0., 0., 0.) {}
    
    void setLightPos(const Eigen::Vector3f& pos) { fLightPos = pos; }
    void setLightPos(float x, float y, float z) { setLightPos(Eigen::Vector3f(x, y, z)); }
    Eigen::Vector3f lightPos() const { return fLightPos; }
    
    void addObject(Object* obj);
    void drawObjects();
    void castShadow();
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  private:
    void castShadowPass(const std::vector< std::vector<bool> >& visible);
    
    std::vector<const Object*> fObj;
    Eigen::Vector3f fLightPos;
};

#endif
