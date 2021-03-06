#!/usr/bin/python
from __future__ import division
import ode
import math

def motorResponse(speed):
    if speed > 15:
        return .05 * (speed - 15)
    elif speed > -15:
        return 0.
    else:
        return .05 * (speed + 15)
        
error_int = 0
def getMotor(vSet, v):
    global error_int
    
    error = (vSet - v)
    error_int += error
    
    mot = 50 * error + 25 * error_int
    
    return mot

def getAccel(xDot, phiDot, x, phi):
    k = 2
    g = 9.81
    l = 0.35
    
    a1 = 4*l/g*k**3
    a2 = 4*k*l * (1 + l/g*k**2)
    a3 = l/g * k**4
    a4 = 6 * k**2 * l + g + l**2/g*k**4
    
    a = a1 * xDot + a2 * phiDot + a3 * x + a4 * phi
    
    return a

phiIni = .3

world = ode.World()
world.setGravity( (0., 0., -9.81) )

ball = ode.Body(world)
mb = ode.Mass()
mb.setSphereTotal(.1, 0.1)
ball.setMass(mb)
ball.setPosition((.35 * math.sin(phiIni), 0., .35 * math.cos(phiIni)))

car = ode.Body(world)
mc = ode.Mass()
mc.setSphereTotal(1.0, 0.1)
car.setMass(mc)
car.setPosition((0., 0., 0.))

j1 = ode.HingeJoint(world)
j1.attach(ball, car)
j1.setAnchor((0., 0., 0.))
j1.setAxis((0., 1., 0. ))

j2 = ode.SliderJoint(world)
j2.attach(car, ode.environment)
j2.setAxis((1., 0., 0.))
j2.setParam(ode.ParamFMax, 0.1)

t = 0.0
dt = 0.01
vSet = 0

while t < 100.0:
    x,y,z = car.getPosition()
    vx,vy,vz = car.getLinearVel()
    phi = j1.getAngle() + phiIni
    phiDot = j1.getAngleRate()
    
    vSet += getAccel(vx, phiDot, x, phi) * dt
    mot = getMotor(vSet, vx)
    
    j2.setParam(ode.ParamVel, motorResponse(mot))
    
    print "%.6f %.6f %.6f %.6f %.6f" % (t, vx, phiDot, x, phi)

    world.step(dt)
    t += dt
