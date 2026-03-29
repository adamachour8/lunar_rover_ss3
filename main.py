import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation.terrain_generator import *
from perception.ransac import ransac
from perception.DBSCAN import dbscan
from perception.filtration import filtrer

import numpy as np
import matplotlib.pyplot as plt

# VARIABLES -----------------------------------------------------------------

THRESHOLD = 0.08
NOM_FICHIER = "NuagePtsTest1-6.csv"
EPS = 0.25
MIN_SAMPLES = 8

# PROGRAMME -----------------------------------------------------------------

points = generer_terrain("simulation/" + NOM_FICHIER)
terrain_non_navigable, sol, _ = ransac(points, THRESHOLD)
clusters = dbscan(terrain_non_navigable, EPS, MIN_SAMPLES)
objets_interet, obstacles = filtrer(clusters)

for obj in objets_interet:
    print(obj)
for obj in obstacles:
    print(obj)

# PLOT ----------------------------------------------------------------------

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