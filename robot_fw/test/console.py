#!/usr/bin/python
from __future__ import division
import struct
import serdecode

serdecode.resync()

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=h", s)
    print x

