#!/usr/bin/python
import struct
import serdecode

serdecode.resync()

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=HHH", s)
    print x

