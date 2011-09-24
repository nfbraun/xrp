#!/usr/bin/python
from __future__ import division
import struct
import serdecode
import math

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

class Delay:
    def __init__(self, l):
        self.l = l
        self.data = [0.] * l
    
    def add(self, x):
        del self.data[0]
        self.data.append(x)
    
    def value(self):
        return self.data[0]

phi_dot_avg = MovingAverage(10)
omega_avg = MovingAverage(10)
gx_avg = MovingAverage(10)
gy_avg = MovingAverage(20)

v_delay = Delay(20)

omega_sum = 0.
offset = 0.

while True:
    s = serdecode.read_frame()
    
    # (phi, omega, speed_0, gx, gy, gz, omega_int) = struct.unpack("=hhhhhhi", s)
    # gx_avg.add(gx)
    # v_delay.add(speed_0)
    # omega_sum += omega
    # omega_sum -= 0.001 * omega_sum
    
    # phi_1 = gx_iavg/(136*32)*180./math.pi*67.6  # slow estimate
    # phi_1 = gx_iavg*0.88  # slow estimate
    # phi_2 = omega_int            # fast estimate
    
    # offset = 0.99*offset + 0.01*(phi_1 - phi_2)
    # phi_est = phi_2 + offset
    (speed, user_speed, pos, user_pos) = struct.unpack("=hhii", s)
    outfile.write(struct.pack("=ffff", speed, user_speed, pos, user_pos))
    outfile.flush()

outfile.close()
