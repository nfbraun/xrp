#!/usr/bin/python
from __future__ import division
import ode
import math

def radToDeg(phi):
    return (phi / math.pi) * 180

def desiredVelocity(t):
    if t < 20:
        return 1
    elif t < 40:
        return 0
    elif t < 60:
        return -1
    else:
        return 0

def getForce(t, phi, phiDot, vCur):
    vSet = desiredVelocity(t)
    return (200.0 * (phi + 5 * phiDot + .1*(vCur - desiredVelocity(t))), 0., 0.)

world = ode.World()
world.setGravity( (0., 0., -9.81) )

ball = ode.Body(world)
mb = ode.Mass()
mb.setSphere(1000.0, 0.1)
ball.setMass(mb)
ball.setPosition((0., 0., 1.))

car = ode.Body(world)
mc = ode.Mass()
mc.setSphere(1000.0, 0.1)
car.setMass(mc)
car.setPosition((0., 0., 0.))

j1 = ode.HingeJoint(world)
j1.attach(ball, car)
j1.setAnchor((0., 0., 0.))
j1.setAxis((0., 1., 0. ))

j2 = ode.SliderJoint(world)
j2.attach(car, ode.environment)
j2.setAxis((1., 0., 0.))

ball.setPosition((math.sin(0), 0., math.cos(0)))

t = 0.0
dt = 0.001

while t < 100.:
    x,y,z = car.getPosition()
    vx,vy,vz = car.getLinearVel()
    phi = j1.getAngle()
    phiDot = j1.getAngleRate()

    car.addForce(getForce(t, phi, phiDot, vx))

    print "%.6f %.6f %.6f %.6f" % (t, x, vx, radToDeg(phi))

    world.step(dt)
    t += dt
