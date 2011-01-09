#!/usr/bin/python
from __future__ import division
import ode
import math
import random

class Sys:
    G = 9.81      # m * s^-2
    R = 0.083/2   # m
    L = 0.256     # m
    M_B = 2.14    # kg
    I_B = 0.1175  # kg m^2

def get_accel(phidot, phi, psidot, psi):
    u = (4570, 192, 5190, 10)
    return phidot*u[0] + phi*u[1] + psidot*u[2] + psi*u[3]

def near_callback(args, geom1, geom2):
    contacts = ode.collide(geom1, geom2)
    
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.)
        c.setMu(100.)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

world = ode.World()
world.setGravity( (0., 0., -Sys.G) )

space = ode.SimpleSpace()

wheel = ode.Body(world)
mw = ode.Mass()
# BUG(?): setSphereTotal() does not seem to work
mw.setSphere(0.1/(4./3.*math.pi*Sys.R**3), Sys.R)
wheel.setMass(mw)
wheel.setPosition((0., 0., 0.))

wheelG = ode.GeomCylinder(space, Sys.R, 1.)
wheelG.setBody(wheel)
wheelG.setQuaternion((1./math.sqrt(2.), 1./math.sqrt(2.), 0., 0.))

body = ode.Body(world)
mb = ode.Mass()
mb.setParameters(Sys.M_B, 0., 0., 0., 1., Sys.I_B, 1., 0., 0., 0.)
body.setMass(mb)
body.setPosition((Sys.L*math.sin(.1), 0., Sys.L*math.cos(.1)))

j1 = ode.BallJoint(world)
j1.setAnchor((0., 0., 0.))
j1.attach(wheel, body)

motor = ode.AMotor(world)
motor.attach(wheel, body)
motor.setNumAxes(1)
motor.setAxis(0, 0, (0., 1., 0.))
motor.setParam(ode.ParamFMax, 1.)
motor.setParam(ode.ParamVel, 0.)

floor = ode.GeomPlane(space, (0., 0., 1.), -Sys.R)
contactgroup = ode.JointGroup()

t = 0.0
dt = 0.01

target_v = 0
motor_v = 0

old_phi = int(0.1 * (180/math.pi)*685)
old_psi = 0

while t < 10.:
    bx,by,bz = body.getPosition()
    wx,wy,wz = wheel.getPosition()
    bvx,bvy,bvz = body.getLinearVel()
    wvx,wvy,wvz = wheel.getLinearVel()
    box,boy,boz = body.getAngularVel()
    wox,woy,woz = wheel.getAngularVel()
    
    p = (t, boy, math.atan2(bx-wx,bz-wz), woy, wx / Sys.R, target_v/(2**20))
    print ("%.6f " * len(p)) % p
    
    phi = int((math.atan2(bx-wx,bz-wz))*(180/math.pi)*685 + 1*685*(random.random()-.5))
    psi = int((wx / Sys.R)*(512/math.pi))
    # phi = math.atan2(bx-wx, bz-wz)*(180/math.pi)*685
    # psi = wx/Sys.R*512/math.pi
    # phi_dot = boy * (180/math.pi)*685 * dt
    # psi_dot = woy * (512/math.pi) * dt
    target_v += get_accel(phi-old_phi, phi, int(target_v/(2**20)), psi)
    if int(target_v / 2**20) > motor_v:
        motor_v += .5
    elif int(target_v / 2**20) < motor_v:
        motor_v -= .5
    motor.setParam(ode.ParamVel, motor_v/512*math.pi*100)
    old_phi = phi
    old_psi = psi
    # target_v += get_accel(boy, math.atan2(bx-wx,bz-wz), woy, wx / Sys.R) * dt
    # motor.setParam(ode.ParamVel, target_v)
    
    for i in range(0, 10):
        space.collide((world, contactgroup), near_callback)
        world.step(dt/10.)
        contactgroup.empty()
        t += dt/10.
