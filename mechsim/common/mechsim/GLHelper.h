#ifndef MSIM_GLHELPER_H
#define MSIM_GLHELPER_H

#include <Eigen/Dense>
#include <GL/gl.h>
#include <ode/ode.h>
#include <cmath>

namespace GL {

using Eigen::Vector3d;

void drawSphere(double r, Vector3d p);
void drawDiscSegment(double r, double h, double alpha);
void drawCheckerboardFloor();
void drawCheckerboardFloorOutline();
void drawTube(double r, Vector3d p1, Vector3d p2);
void drawODEBox(dGeomID id, double lx, double ly, double lz);
void drawBox(Vector3d p1, Vector3d p2);
void drawUnitCube();

void shadowsBeginFloor();
void shadowsBeginObjects(Vector3d floor_n, double floor_d);
void shadowsEnd();

const double RAD_TO_DEG = 180. / M_PI;

namespace internal {
inline void Translate(double x, double y, double z)
    { glTranslated(x, y, z); }
inline void Translate(float x, float y, float z)
    { glTranslatef(x, y, z); }
}
template <typename Derived>
inline void Translate(const Eigen::MatrixBase<Derived>& v)
    { internal::Translate(v.x(), v.y(), v.z()); }

namespace internal {
inline void Vertex3(double x, double y, double z)
    { glVertex3d(x, y, z); }
inline void Vertex3(float x, float y, float z)
    { glVertex3f(x, y, z); }
}
template <typename Derived>
inline void Vertex3(const Eigen::MatrixBase<Derived>& v)
    { internal::Vertex3(v.x(), v.y(), v.z()); }

namespace internal {
inline void Normal3(double x, double y, double z)
    { glNormal3d(x, y, z); }
inline void Normal3(float x, float y, float z)
    { glNormal3f(x, y, z); }
}
template <typename Derived>
inline void Normal3(const Eigen::MatrixBase<Derived>& v)
    { internal::Normal3(v.x(), v.y(), v.z()); }

template <typename Derived>
void Rotate(const Eigen::QuaternionBase<Derived>& q)
{
    Eigen::Affine3d transform(q);
    glMultMatrixd(transform.data());
}

template <typename Scalar>
void Rotate(const Eigen::AngleAxis<Scalar>& aa)
{
    Eigen::Affine3d transform(aa);
    glMultMatrixd(transform.data());
}

} // end namespace GL

#endif
