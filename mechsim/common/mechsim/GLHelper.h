#ifndef MSIM_GLHELPER_H
#define MSIM_GLHELPER_H

#include "Vector.h"
#include "Rotation.h"
#include <GL/gl.h>
#include <ode/ode.h>
#include <cmath>

namespace GL {

void drawSphere(double r, Vector3 p);
void drawDiscSegment(double r, double h, double alpha);
void drawCheckerboardFloor();
void drawTube(double r, Vector3 p1, Vector3 p2);
void drawODEBox(dGeomID id, double lx, double ly, double lz);
void drawBox(Vector3 p1, Vector3 p2);
void drawUnitCube();

void shadowsBeginFloor();
void shadowsBeginObjects(Vector3 floor_n, double floor_d);
void shadowsEnd();

const double RAD_TO_DEG = 180. / M_PI;

inline void Translate(const Vector3& v)
    { glTranslated(v.x(), v.y(), v.z()); }
inline void Translate(const Vector3f& v)
    { glTranslatef(v.x(), v.y(), v.z()); }

void Rotate(const Rotation& r);

} // end namespace GL

#endif
