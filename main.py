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

THRESHOLD = 0.1 # Distance du plan pour que le point soit considéré comme en faisant partie ou non
NOM_FICHIER = "NuagePtsTest1-6.csv" # Nom du fichier du nuage de points (dans "simulation")
EPS = 0.3 # Distance d'un point à un autre pour considérer qu'il s'agit du même cluster
MIN_SAMPLES = 6 # Minimum de points pour être considéré comme un cluster

# PROGRAMME -----------------------------------------------------------------

points = generer_terrain("simulation/" + NOM_FICHIER)
obstacles = ransac(points, THRESHOLD)
clusters = dbscan(obstacles, EPS, MIN_SAMPLES)


# TESTS (TEMPORAIRE) --------------------------------------------------------

# #Imprimer en ordre sexy (DBSCAN)
# for label, cluster_points in clusters.items():
#     print(f"Cluster {label}: {len(cluster_points)} points")
#     print(cluster_points)

# Tester graphiquement (RANSAC) + DBSCAN
ranges = np.array([
    points[:,0].max() - points[:,0].min(),
    points[:,1].max() - points[:,1].min(),
    points[:,2].max() - points[:,2].min()
])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for label, cluster_points in clusters.items():
    ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_box_aspect(ranges)
plt.show()