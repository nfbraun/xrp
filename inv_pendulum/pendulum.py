#!/usr/bin/python
from __future__ import division
import ode
import math

def radToDeg(phi):
    return (phi / math.pi) * 180

# def desiredVelocity(t):
#     if t < 20:
#         return 1
#     elif t < 40:
#         return 0
#     elif t < 60:
#         return -1
#     else:
#         return 0

def motorResponse(speed):
    if speed > 15:
        return .05 * (speed - 15)
    elif speed > -15:
        return 0.
    else:
        return .05 * (speed + 15)

error_int = 0
def getMotor(t, phi, phiDot, pos, v):
    # Units: [pos] = 1/4096 m
    #        [phi] = (2*pi)/4096 rad = 1/652 rad

    global error_int
    
    # phi += .01

    # return .4 * phi + .2 * phiDot + 1.02 * v + .001 * pos
    # error =  20. * (.4 * phi + .005 * pos + .4 * phiDot + 0.02 * v)
    # error = 500 * phi + 50000 * phiDot + pos + 400 * v
    error = 500 * phi + 35000 * phiDot + pos + 400 * v
    error_int += error
    
    return (error + 3 * error_int) // 100000

phiIni = .2

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
j2.setParam(ode.ParamFMax, 5.)

t = 0.0
dt = 0.01

oldPhi = 0
oldX = 0

while t < 100.0:
    x,y,z = car.getPosition()
    vx,vy,vz = car.getLinearVel()
    phi = j1.getAngle() + phiIni
    phiDot = j1.getAngleRate()
    
    phiInt = int(phi*652.)
    xInt = int(x * 4096.)
    
    mot = getMotor(t, phiInt, phiInt - oldPhi, xInt, xInt - oldX)
    oldPhi = phiInt
    oldX = xInt
    
    j2.setParam(ode.ParamVel, motorResponse(mot))
    
    print "%.6f %.6f %.6f %.6f" % (t, x, vx, radToDeg(phi))

    world.step(dt)
    t += dt
