#!/usr/bin/python
import ode
import math

world = ode.World()
world.setGravity( (0., 0., -9.81) )

ball = ode.Body(world)
m = ode.Mass()
m.setSphere(1000.0, 0.1)
ball.setMass(m)
ball.setPosition( (-1., 0., -1.) )

j1 = ode.HingeJoint(world)
j1.attach(ball, ode.environment)
j1.setAnchor( (0.,0.,0.) )
j1.setAxis( (0.,1.,0.) )
j1.setParam(ode.ParamLoStop, -math.pi/4.)

t = 0.0
dt = 0.01

while t < 5.:
    x,y,z = ball.getPosition()
    vx,vy,vz = ball.getLinearVel()
    phi = math.atan2(-x, -z)

    print "%.4f %.4f" % (t, phi)

    world.step(dt)
    t += dt
