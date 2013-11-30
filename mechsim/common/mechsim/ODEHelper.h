#ifndef MSIM_ODEHELPER_H
#define MSIM_ODEHELPER_H

#include <ode/ode.h>
#include <Eigen/Geometry>

namespace ODE {
    typedef Eigen::Matrix<dReal, 3, 1> _Vector3;
    
    inline void BodySetPosition(dBodyID id, const _Vector3& r)
        { dBodySetPosition(id, r.x(), r.y(), r.z()); }
    inline void BodySetLinearVel(dBodyID id, const _Vector3& v)
        { dBodySetLinearVel(id, v.x(), v.y(), v.z()); }
    inline void BodySetAngularVel(dBodyID id, const _Vector3& o)
        { dBodySetAngularVel(id, o.x(), o.y(), o.z()); }
    inline void BodyAddForce(dBodyID id, const _Vector3& f)
        { dBodyAddForce(id, f.x(), f.y(), f.z()); }
    inline void BodyAddTorque(dBodyID id, const _Vector3& t)
        { dBodyAddTorque(id, t.x(), t.y(), t.z()); }
        
    inline void JointSetHingeAnchor(dJointID j, const _Vector3& r)
        { dJointSetHingeAnchor(j, r.x(), r.y(), r.z()); }
    inline void JointSetHingeAxis(dJointID j, const _Vector3& a)
        { dJointSetHingeAxis(j, a.x(), a.y(), a.z()); }
    inline void JointSetUniversalAnchor(dJointID j, const _Vector3& r)
        { dJointSetUniversalAnchor(j, r.x(), r.y(), r.z()); }
    inline void JointSetUniversalAxis1(dJointID j, const _Vector3& a)
        { dJointSetUniversalAxis1(j, a.x(), a.y(), a.z()); }
    inline void JointSetUniversalAxis2(dJointID j, const _Vector3& a)
        { dJointSetUniversalAxis2(j, a.x(), a.y(), a.z()); }
    inline void GeomSetOffsetPosition(dGeomID g, const _Vector3& r)
        { dGeomSetOffsetPosition(g, r.x(), r.y(), r.z()); }
    
    template<typename Derived>
    inline void BodySetQuaternion(dBodyID b, const Eigen::QuaternionBase<Derived>& q)
    {
        // Note that the order in which quaternion coefficients are stored
        // differs between Eigen and ODE!
        dQuaternion tmp = { q.w(), q.x(), q.y(), q.z() };
        dBodySetQuaternion(b, tmp);
    }
    
    template<typename Derived>
    inline void GeomSetOffsetQuaternion(dGeomID g, const Eigen::QuaternionBase<Derived>& q)
    {
        // Note that the order in which quaternion coefficients are stored
        // differs between Eigen and ODE!
        dQuaternion tmp = { q.w(), q.x(), q.y(), q.z() };
        dGeomSetOffsetQuaternion(g, tmp);
    }
    
    inline Eigen::Quaternion<dReal> QuatFromArray(const dReal* data)
    {
        // Note that the order in which quaternion coefficients are stored
        // in the quaternion class differs from the order in the constructor
        // (see manual)
        return Eigen::Quaternion<dReal>(data[0], data[1], data[2], data[3]);
    }
    
    inline _Vector3 VectorFromArray(const dReal* data)
        { return Eigen::Map< const Eigen::Matrix<dReal, 3, 1> >(data); }
    
    inline _Vector3 BodyGetPosition(dBodyID b)
        { return VectorFromArray(dBodyGetPosition(b)); }
    
    inline Eigen::Quaternion<dReal> BodyGetQuaternion(dBodyID b)
        { return QuatFromArray(dBodyGetQuaternion(b)); }
    
    inline _Vector3 BodyGetLinearVel(dBodyID b)
        { return VectorFromArray(dBodyGetLinearVel(b)); }
    
    inline _Vector3 BodyGetAngularVel(dBodyID b)
        { return VectorFromArray(dBodyGetAngularVel(b)); }
    
    inline _Vector3 JointGetHingeAxis(dJointID j)
        { dVector3 axis; dJointGetHingeAxis(j, axis); return VectorFromArray(axis); }
    
    inline _Vector3 JointGetUniversalAnchor(dJointID j)
        { dVector3 anchor; dJointGetUniversalAnchor(j, anchor); return VectorFromArray(anchor); }
    
    inline _Vector3 JointGetUniversalAxis1(dJointID j)
        { dVector3 axis; dJointGetUniversalAxis1(j, axis); return VectorFromArray(axis); }
    
    inline _Vector3 JointGetUniversalAxis2(dJointID j)
        { dVector3 axis; dJointGetUniversalAxis2(j, axis); return VectorFromArray(axis); }
    
    
} // end namespace ODE

#endif
