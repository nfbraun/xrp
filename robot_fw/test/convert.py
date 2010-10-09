#!/usr/bin/python
import struct
import serdecode

outfile = open("softscope.fifo", "w")

serdecode.resync()

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=hi", s)
    outfile.write(struct.pack("=ff", x[0], x[1]))
    outfile.write(struct.pack("=ff", 0., 0.))
    outfile.flush()

outfile.close()
