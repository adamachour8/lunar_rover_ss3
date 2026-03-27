import numpy as np
from sklearn.cluster import DBSCAN
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from perception.ransac import ransac
from simulation.terrain_generator import generate_terrain

THRESHOLD = 0.05

points = generate_terrain(100,50)
obstacles = ransac(points, THRESHOLD)
db = DBSCAN(eps=0.3, min_samples=10).fit(obstacles)
print(db)