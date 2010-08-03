#!/usr/bin/python
import struct
import serdecode

outfile = open("test.fifo", "w")

serdecode.resync()

while True:
    s = serdecode.read_frame()
    x = struct.unpack("=HHH", s)
    outfile.write(struct.pack("=hhh", x[0], x[1], x[2]))
    outfile.write(struct.pack("=h", 0))
    outfile.flush()

outfile.close()
