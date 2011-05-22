#!/usr/bin/python
from __future__ import division
import math

tbl = []
for x in range(0, 51):
    tbl.append("%d" % math.ceil(30.*math.sin(x/50*math.pi/2)-.5))

print "int8_t sin_lut[] = {",
print ", ".join(tbl),
print "};"
