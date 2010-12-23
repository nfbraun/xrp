#!/usr/bin/python
from __future__ import division
import struct
import serdecode

outfile = open("softscope.fifo", "w")

serdecode.resync()

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=h", s)
    
    outfile.write(struct.pack("=ffff", x[0], 0., 0., 0.))
    outfile.flush()

outfile.close()
