#!/usr/bin/python -i
from __future__ import division

import scipy.linalg as linalg
import scipy.linalg.matfuncs as mf
from scipy import mat

# Random example values
# a1 = .01
# a2 = .01
# a3 = -1
# a4 = 15

# g = 9.81
# l = 0.35

k = -5
g = 9.81
l = 0.35

a1 = -4*l/g*k**3
a2 = -4*k*l * (1 + l/g*k**2)
a3 = l/g * k**4
a4 = 6 * k**2 * l + g + l**2/g*k**4

# print a1, a2, a3, a4

# phiIni = .1

A = mat([[a1,a2,a3,a4],[-a1/l,-a2/l,-a3/l,-a4/l+g/l],[1,0,0,0],[0,1,0,0]])

y0 = mat([[0],[0],[0],[.1]])

# la, v = linalg.eig(A)
# print la

# for i in range(0, 1000):
#     t = i / 100.
#     y = mf.expm(A * t) * y0
#     print "%.6f %.6f %.6f %.6f %.6f" % (t, y[0][0], y[1][0], y[2][0], y[3][0])
