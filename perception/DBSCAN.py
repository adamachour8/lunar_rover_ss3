import numpy as np
from sklearn.cluster import DBSCAN
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from perception.ransac import ransac
from simulation.terrain_generator import generate_terrain

THRESHOLD = 0.05

#Cherche le bon nuage de pts
points = generate_terrain(100,50)
obstacles = ransac(points, THRESHOLD)

#DBSCAN
db = DBSCAN(eps=0.3, min_samples=5).fit(obstacles)

# Separate points by cluster
labels = db.labels_
clusters = {}
for label in set(labels):
    if label == -1:
        continue  # skip noise points
    clusters[label] = obstacles[labels == label]
    
for label, cluster_points in clusters.items():
    print(f"Cluster {label}: {len(cluster_points)} points")
    print(cluster_points)
