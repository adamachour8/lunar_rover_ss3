# IMPORTS -------------------------------------------------------------------

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.terrain_generator import *
from perception.RANSAC import ransac
from perception.DBSCAN import dbscan

import numpy as np
import matplotlib.pyplot as plt

# VARIABLES -----------------------------------------------------------------

THRESHOLD = 0.05

# PROGRAMME -----------------------------------------------------------------

points = generate_lunar_terrain(2000)
obstacles = ransac(points, THRESHOLD)
clusters = dbscan(obstacles)


# TESTS (TEMPORAIRE) --------------------------------------------------------

#Imprimer en ordre sexy (DBSCAN)
for label, cluster_points in clusters.items():
    print(f"Cluster {label}: {len(cluster_points)} points")
    print(cluster_points)

# Tester graphiquement (RANSAC) + DBSCAN
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for label, cluster_points in clusters.items():
    ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()