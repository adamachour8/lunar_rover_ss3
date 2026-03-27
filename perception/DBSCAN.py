import numpy as np
from sklearn.cluster import DBSCAN
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from perception.ransac import ransac
from simulation.terrain_generator import generate_lunar_terrain
import matplotlib.pyplot as plt

THRESHOLD = 0.05

#Cherche le bon nuage de pts
points = generate_lunar_terrain(2000)
obstacles = ransac(points, THRESHOLD)

#DBSCAN
db = DBSCAN(eps=0.3, min_samples=5).fit(obstacles)

# Separate points by cluster
labels = db.labels_
clusters = {}


for label in set(labels):
    if label == -1:
        continue  # skip noise points
    clusters[label] = obstacles[labels == label] #clusters est un dictionnaire dont la cle est le numero du clump et la valeur est un np.array de dim (N,3)

#Imprimer en ordre sexy
# for label, cluster_points in clusters.items():
#     print(f"Cluster {label}: {len(cluster_points)} points")
#     print(cluster_points)

#-----------------------------PLOT-------------------------------------
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(clusters[0][:, 0], clusters[0][:, 1], clusters[0][:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()