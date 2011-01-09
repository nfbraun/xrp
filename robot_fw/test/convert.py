#!/usr/bin/python
from __future__ import division
import struct
import serdecode

outfile = open("softscope.fifo", "w")

serdecode.resync()

class MovingAverage:
    def __init__(self, l):
        self.l = l
        self.data = [0.] * l
    
    def add(self, x):
        del self.data[0]
        self.data.append(x)
    
    def value(self):
        return sum(self.data) / self.l

phi_dot_avg = MovingAverage(10)
omega_avg = MovingAverage(10)

while True:
    s = serdecode.read_frame()
    (target_speed, phi_dot, x, phi, speed_0, speed_1, omega, idle_cnt, error_int_0, error_int_1) = \
        struct.unpack("=ihihhhhhhh", s)
    
    phi_dot_avg.add(phi_dot)
    omega_avg.add(omega)
    
    outfile.write(struct.pack("=ffff", target_speed/2**20, speed_0, error_int_0, idle_cnt))
    outfile.flush()

outfile.close()
