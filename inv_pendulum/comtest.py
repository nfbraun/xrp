#!/usr/bin/python
import ode
import math

d = 3.
m = 7.
J = 5.
g = 10.

# Should give an (approximately) harmonic oscillation with
# \omega = \sqrt{\frac{d m g}{J + md^2}}

world = ode.World()
world.setGravity( (0., 0., -g) )

ball = ode.Body(world)
mass = ode.Mass()
mass.setParameters(m, 0., 0., 0., 1000., J, 1000., 0., 0., 0.)
ball.setMass(mass)
ball.setPosition( (d*math.sin(.1), 0., -d*math.cos(.1)) )

j1 = ode.HingeJoint(world)
j1.attach(ball, ode.environment)
j1.setAnchor( (0.,0.,0.) )
j1.setAxis( (0.,1.,0.) )
# j1.setParam(ode.ParamVel, 0.)
# j1.setParam(ode.ParamFMax, 1.)

t = 0.0
dt = 0.01

while t < 30.:
	x,y,z = ball.getPosition()
	vx,vy,vz = ball.getLinearVel()
	phi = j1.getAngle()
	
	print "%.4f %.4f" % (t, phi)

	world.step(dt)
	t += dt
