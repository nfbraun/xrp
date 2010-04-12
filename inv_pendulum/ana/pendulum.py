#!/usr/bin/python
from __future__ import division
import ode
import math

def getAccel(xDot, phiDot, x, phi):
    k = 2
    g = 9.81
    l = 0.35
    
    a1 = 4*l/g*k**3
    a2 = 4*k*l * (1 + l/g*k**2)
    a3 = l/g * k**4
    a4 = 6 * k**2 * l + g + l**2/g*k**4
    
    return a1 * xDot + a2 * phiDot + a3 * x + a4 * phi

phiIni = .1

world = ode.World()
world.setGravity( (0., 0., -9.81) )

ball = ode.Body(world)
mb = ode.Mass()
mb.setSphereTotal(1.0, 0.01)
mb.adjust(1.)
ball.setMass(mb)
ball.setPosition((.35 * math.sin(phiIni), 0., .35 * math.cos(phiIni)))

car = ode.Body(world)
mc = ode.Mass()
mc.setSphereTotal(1.0, 0.01)
mc.adjust(1.)
car.setMass(mc)
car.setPosition((0., 0., 0.))

j1 = ode.HingeJoint(world)
j1.attach(ball, car)
j1.setAnchor((0., 0., 0.))
j1.setAxis((0., 1., 0. ))

j2 = ode.SliderJoint(world)
j2.attach(car, ode.environment)
j2.setAxis((1., 0., 0.))
j2.setParam(ode.ParamFMax, 10.)

t = 0.0
dt = 0.01
vSet = 0.

while t < 10.0:
    x,y,z = car.getPosition()
    vx,vy,vz = car.getLinearVel()
    phi = j1.getAngle() + phiIni
    phiDot = j1.getAngleRate()
    
    vSet += getAccel(vx, phiDot, x, phi)*dt
    j2.setParam(ode.ParamVel, vSet)
    
    print "%.6f %.6f %.6f %.6f %.6f" % (t, vx, phiDot, x, phi)

    world.step(dt)
    t += dt
