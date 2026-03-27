import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from perception.ransac import ransac
from simulation.terrain_generator import generate_terrain

THRESHOLD = 0.05

points = generate_terrain(100,50)
obstacles = ransac(points, THRESHOLD)
print(obstacles)