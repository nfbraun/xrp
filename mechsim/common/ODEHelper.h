#ifndef __ODEHELPER_H__
#define __ODEHELPER_H__

#include <ode/ode.h>

namespace ODE {
    inline void BodySetLinearVel(dBodyID id, const Vector3& v)
        { dBodySetLinearVel(id, v.x(), v.y(), v.z()); }
    inline void BodySetAngularVel(dBodyID id, const Vector3& o)
        { dBodySetAngularVel(id, o.x(), o.y(), o.z()); }
    inline void JointSetHingeAnchor(dJointID j, Vector3 r)
        { dJointSetHingeAnchor(j, r.x(), r.y(), r.z()); }
    inline void JointSetHingeAxis(dJointID j, Vector3 a)
        { dJointSetHingeAxis(j, a.x(), a.y(), a.z()); }
    inline void GeomSetOffsetPosition(dGeomID g, Vector3 r)
        { dGeomSetOffsetPosition(g, r.x(), r.y(), r.z()); }
} // end namespace ODE

#endif
