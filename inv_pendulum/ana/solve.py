#!/usr/bin/python
import sympy
from sympy import var, solve

delta,I = var("delta I")

m = 2.14  # kg
g = 9.81 # m/s**2
omega1 = 0.0451 * 101.4  # s**(-1)
r1 = 0.50 # m
omega2 = 0.0272 * 101.4 # s**(-1)
r2 = 0.30 # m

Istar = solve(omega1**2 - (m*g*(r1-delta))/(m*(r1-delta)**2 + I), I)
deltastar = solve(omega2**2 - (m*g*(r2-delta))/(m*(r2-delta)**2 + Istar[0]), delta)
Istar = solve(omega1**2 - (m*g*(r1-deltastar[0]))/(m*(r1-deltastar[0])**2 + I), I)

print deltastar[0], Istar[0]
