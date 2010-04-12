#!/usr/bin/python
from __future__ import division

k = 1.2
g = 9.81
l = 0.50

a1 = 4*l/g*k**3
a2 = 4*k*l * (1 + l/g*k**2)
a3 = l/g * k**4
a4 = 6 * k**2 * l + g + l**2/g*k**4

scale = 2**20
a1 = (a1 * (100/4096) * 4096 / 10000 * scale)
a2 = (a2 * (100/652) * 4096 / 10000 * scale)
a3 = (a3 * (1/4096) * 4096 / 10000 * scale)
a4 = (a4 * (1/652) * 4096 / 10000 * scale)

print "#define A1 %d" % int(a1)
print "#define A2 %d" % int(a2)
print "#define A3 %d" % int(a3)
print "#define A4 %d" % int(a4)
