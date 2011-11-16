#!/usr/bin/python
import math
import scipy
import scipy.optimize
from scipy import linspace

H = 0.5
x_F = 1

def func(x):
    return (math.cos(x) - 1)/(x - math.sin(x))

if H < -1e-5:
    raise RuntimeError, "H must not be negative"
elif H < 1e-5:
    phi_F = 2*math.pi
    R = x_F/(2*math.pi)
else:
    phi_F = scipy.optimize.brentq(lambda x: func(x) + H/x_F, 1e-5, 2*math.pi,
        disp=True, full_output=False)
    R = -H/(math.cos(phi_F)-1)

for phi in linspace(0., phi_F, 1000):
    x = R*(phi - math.sin(phi))
    y = R*(math.cos(phi) - 1.)
    print "%f %f" % (x,y)
