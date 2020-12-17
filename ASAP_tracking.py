#!/usr/bin/env python
import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

k0 = 1.4
k1 = 0.5
k2 = 4
k3 = 2.5
a = np.matrix([0, 0, 1]).transpose()
rho = 2.0

# Agent starting locations
agents = [[5, 1, 7],
          [3, 0, 6],
          [0, 4, 8],
          [0, 10, 0],
          [10, 0 , 0],
          [0, 20, 0]]
n = len(agents)

# Position of threat
start_threat = [3, 3, 7]

# Ring topology adjacency matrix
A = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        offset = abs(i - j) % n
        if offset == 1 or offset == n - 1:
            A[i,j] = 1

# Desired phasing angle between agents (thetaij may be different from thetaik)
thetaij = 2 * math.pi / n

# Desired inter-agent distance
dij = 2 * rho * math.sin(thetaij / 2)

# Matrix of inter-agent distances
D = dij*A

def flatten_array(input):
    return [i for element in input for i in element]

# Rearranges a flat array into an array of 3-arrays: [1 2 3 4 5 6] -> [[1 2 3][4 5 6]]
def raze_array(input):
    output = []
    for i in range(len(input) / 3):
        output.append(input[i*3:i*3+3])

    return output

def rollercoaster(t):
    return [math.sin((2 * math.pi / 50) * t),
           math.cos((2 * math.pi / 50) * t) * 0.75,
           math.sin((6 * math.pi / 50) * t)]

def agent_velocities(agents, target, target_velocity):
    velocities = []


    pt = np.matrix(target).transpose()

    a_a_transpose = a * a.transpose()

    for i in range(len(agents)):

        p_i = np.matrix(agents[i]).transpose()

        pit = p_i - pt

        DeliV1 = a_a_transpose * pit

        # Orthogonal projection matrix of a
        Pa = np.eye(3, 3) - a_a_transpose
        phii = (pit / np.linalg.norm(pit))
        phiia = (Pa * phii) / np.linalg.norm(Pa * phii)

        DeliV2 = (np.linalg.norm(Pa * pit) - rho) * phiia

        DeliV3 = np.matrix([0.0, 0.0, 0.0])

        for j in range(n):
            pj = np.matrix(agents[j]).transpose()
            pjt = pj - pt
            phij = pjt / np.linalg.norm(pjt)
            phija = Pa * phij / np.linalg.norm(Pa*phij)

            gammaij = A[i,j] * ((np.linalg.norm(phiia - phija) ** 2) - ((D[i,j] ** 2) / (rho ** 2))) * np.cross(a.transpose(), phiia.transpose()) * (phiia - phija)

            DeliV3 += (1 / np.linalg.norm(Pa * pit)) * gammaij * np.cross(a.transpose(), phiia.transpose())

        ui1 = -k1 * DeliV1
        ui2 = (-k2 * DeliV2) + (k0 * np.linalg.norm(Pa * pit) * np.cross(a.transpose(), phiia.transpose()).transpose())
        ui3 = -k3 * np.linalg.norm(Pa * pit)*DeliV3.transpose()

        ptdot = np.matrix(target_velocity).transpose()

        ui = ui1 + ui2 + ui3 + ptdot

        velocities.append(np.asarray(ui).flatten())

    return velocities

def model(input, t):
    targets = raze_array(input)
    target = targets[-1]

    target_velocity = rollercoaster(t)
    velocities = agent_velocities(targets[:-1], target, target_velocity)

    return flatten_array(velocities) + target_velocity

t = np.linspace(0, 30, 500)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_aspect('equal')

combined_start = flatten_array(agents) + start_threat

results = odeint(model, combined_start, t)

def plotPart(ax, results, index, color, linestyle):
    ax.plot3D(results[:, (index * 3)], results[:, (index * 3) + 1], results[:, (index * 3) + 2], color = color, linestyle=linestyle)
    markersize = 300
    ax.scatter(results[0, (index * 3)], results[0, (index * 3) + 1], results[0, (index * 3) + 2], color = color, marker = 'o', s = markersize)
    middle = len(results) / 2
    ax.scatter(results[middle, (index * 3)], results[middle, (index * 3) + 1], results[middle, (index * 3) + 2], color = color, marker = '>', s = markersize)
    last = len(results) - 1
    ax.scatter(results[last, (index * 3)], results[last, (index * 3) + 1], results[last, (index * 3) + 2], color = color, marker='X', s = markersize)

styles = ['blue', 'red', 'limegreen', 'purple', 'orange', 'green']

for i in range(len(agents)):
    plotPart(ax, results, i, styles[i], '-')

plotPart(ax, results, len(agents), 'k', '-.')

plt.show()