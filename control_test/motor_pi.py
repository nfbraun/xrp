#!/usr/bin/python

def motor_response(x):
    if x < 5:
        return 0.
    else:
        return (x - 5) * .4
        
        
val = 0
desired_speed = 1
int_error = 0

def pi_timestep():
    global val, desired_speed, int_error

    speed = motor_response(val)
    error = desired_speed - speed
    int_error += error
    
    val = error + int_error
    
for i in range(0,20):
    pi_timestep()
    print val, motor_response(val)
    
desired_speed = 0
for i in range(0,20):
    pi_timestep()
    print val, motor_response(val)

