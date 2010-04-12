#!/usr/bin/python
from __future__ import division
import ode
import math

phiIni = 1.

world = ode.World()
world.setGravity( (0., 0., 9.81) )

ball = ode.Body(world)
mb = ode.Mass()
mb.setSphere(1., 0.01)
mb.adjust(1.)
ball.setMass(mb)
ball.setPosition((1. * math.sin(phiIni), 0., 1. * math.cos(phiIni)))

car = ode.Body(world)
mc = ode.Mass()
mc.setSphere(1., 0.01)
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

t = 0.0
dt = 0.01

while t < 10.0:
    x,y,z = car.getPosition()
    vx,vy,vz = car.getLinearVel()
    phi = j1.getAngle() + phiIni
    phiDot = j1.getAngleRate()
    
    print "%.6f %.6f %.6f %.6f %.6f" % (t, vx, phiDot, x, phi)

    for i in range(0,10):
        car.addForce((3.*math.sin(t), 0., 0.))
        world.step(dt/10.)
        t += dt/10.
