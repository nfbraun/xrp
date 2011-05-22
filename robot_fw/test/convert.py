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
gx_avg = MovingAverage(20)
gy_avg = MovingAverage(20)

v_delay = Delay(40)

while True:
    s = serdecode.read_frame()
    # (target_speed, phi_dot, x, phi, speed_0, speed_1, omega, idle_cnt, error_int_0, error_int_1) = \
    #     struct.unpack("=ihihhhhhhh", s)
    # (target_speed, speed_0, speed_1, error_int_0, error_int_1, motor_0, motor_1) = \
    #     struct.unpack("=ihhhhhh", s)
    #(set_speed_0, error_int_0, motor_0, speed_0, \
    # set_speed_1, error_int_1, motor_1, speed_1) = struct.unpack("=hihhhihh", s)
    # (phi, omega) = struct.unpack("=hh", s)
    # (phi, omega, speed, gx, gy, gz) = struct.unpack("=hhhhhh", s)
    
    # fifo.append(phi)
    # fifo2.append(omega)
    # gx_avg.add(gx)
    # gy_avg.add(gy)
    # v_delay.add(speed)
    
    # phi_dot_avg.add(phi_dot)
    # omega_avg.add(omega)
    
    # outfile.write(struct.pack("=ffff", phi, 4000*fifo2[0], 299*(phi - fifo[0])/10., 0.))
    # outfile.write(struct.pack("=ffff", phi, gy_avg.value(), gx_avg.value(), speed - v_delay.value()))
    # outfile.flush()
    
    #del fifo[0]
    #del fifo2[0]
    
    (dummy, avel, gx, gy, gz) = struct.unpack("=hhhhh", s)
    outfile.write(struct.pack("=ffff", avel, gx, gy, gz))
    outfile.flush()

outfile.close()
