#!/usr/bin/python
from __future__ import division
import math

print "#:1:Foo"
print "#:2:Bar"
print "#:3:Baz"

for t in range(0, 100):
    print t/100, math.sin(t/10),
    print math.sin(t/20),
    print math.sin(t/30),
    print
