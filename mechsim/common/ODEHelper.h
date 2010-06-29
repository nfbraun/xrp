#ifndef __ODEHELPER_H__
#define __ODEHELPER_H__

#include <ode/ode.h>

namespace ODE {
    inline void BodySetLinearVel(dBodyID id, const Vector3& v)
        { dBodySetLinearVel(id, v.x(), v.y(), v.z()); }
    inline void BodySetAngularVel(dBodyID id, const Vector3& o)
        { dBodySetAngularVel(id, o.x(), o.y(), o.z()); }
} // end namespace ODE

#endif
