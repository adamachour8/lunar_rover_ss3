import pyransac3d as pyrsc
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from simulation.terrain_generator import generer_terrain


THRESHOLD = 0.08

def ransac(points, threshold):
    plane1 = pyrsc.Plane()
    best_eq, best_inliers = plane1.fit(points, threshold)

    A, B, C, D = best_eq
    norm = np.sqrt(A**2 + B**2 + C**2)
    distances = np.abs(A*points[:,0] + B*points[:,1] + C*points[:,2] + D) / norm

    sol                   = points[distances <= 0.08]
    terrain_non_navigable = points[distances > 0.08]

    return terrain_non_navigable, sol, best_eq


if __name__ == "__main__":
    points = generer_terrain("simulation/NuagePtsTest1-6.csv")

    terrain_non_navigable, sol, best_eq = ransac(points, THRESHOLD)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(sol[:, 0], sol[:, 1], sol[:, 2], color="blue", s=1, label="Sol")
    ax.scatter(terrain_non_navigable[:, 0], terrain_non_navigable[:, 1], terrain_non_navigable[:, 2], color="red", s=2, label="Non navigable")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()