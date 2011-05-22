#!/usr/bin/python
from __future__ import division
import struct
import serdecode

serdecode.resync()

t = 0

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=hhhhh", s)
    print "%d %d %d" % (t, x[1], x[2])
    t += 1

# t = 0
# while True:
#     s = serdecode.read_frame()
#     x = struct.unpack("=hh", s)
#     print "%d %d %d" % (t, x[0], x[1])
#     t += 1

