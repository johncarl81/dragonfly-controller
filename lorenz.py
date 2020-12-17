#!/usr/bin/env python
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

P = 10
R = 28
B = 8.0/3

def model(input, t):
    x = input[0]
    y = input[1]
    z = input[2]
    dxdt = P * (y - x)
    dydt = (R * x) - y - (x * z)
    dzdt = (x * y) - (B*z)
    return [dxdt, dydt, dzdt]

y0 = [1, 2, 3]
y1 = [1, 1, 3]
y2 = [1, 2, 1]

t = np.linspace(0, 20, 5000)

fig = plt.figure()
ax = plt.axes(projection='3d')

def plotODE(starting, t, ax, style):
    result = odeint(model, starting, t)
    ax.plot3D(result[:,0], result[:,1], result[:,2], style)

plotODE([1, 2, 3], t, ax, 'mediumblue')
plotODE([1, 1, 3], t, ax, 'red')
plotODE([1, 2, 1], t, ax, 'midnightblue')

plt.show()