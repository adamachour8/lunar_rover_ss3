import random
import numpy as np
import matplotlib.pyplot as plt
import pyransac3d as pyrsc
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import RANSAC_THRESHOLD, RANSAC_BOSSEE_MAX, RANSAC_SEED, NOM_FICHIER
from simulation.terrain_generator import generer_terrain


def ransac(points, threshold=RANSAC_THRESHOLD):
    random.seed(RANSAC_SEED)
    np.random.seed(RANSAC_SEED)
    plane = pyrsc.Plane()
    best_eq, _ = plane.fit(points, threshold)

    A, B, C, D = best_eq
    norm = np.sqrt(A**2 + B**2 + C**2)
    dist = (A*points[:,0] + B*points[:,1] + C*points[:,2] + D) / norm

    sol       = points[np.abs(dist) <= threshold]
    au_dessus = points[np.abs(dist) >  threshold]

    dist_au_dessus = np.abs(
        (A*au_dessus[:,0] + B*au_dessus[:,1] + C*au_dessus[:,2] + D) / norm
    )

    terrain_naturel = au_dessus[dist_au_dessus <= RANSAC_BOSSEE_MAX]
    obstacles       = au_dessus[dist_au_dessus >  RANSAC_BOSSEE_MAX]

    return sol, terrain_naturel, obstacles, best_eq


if __name__ == "__main__":
    points = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles, _ = ransac(points)

    print(f"Sol             : {len(sol)} points")
    print(f"Terrain naturel : {len(terrain_naturel)} points")
    print(f"Obstacles       : {len(obstacles)} points")

    fig = plt.figure()
    ax  = fig.add_subplot(projection='3d')
    ax.scatter(sol[:,0],             sol[:,1],             sol[:,2],             color="blue",  s=1, label="Sol")
    ax.scatter(terrain_naturel[:,0], terrain_naturel[:,1], terrain_naturel[:,2], color="green", s=2, label="Terrain naturel")
    ax.scatter(obstacles[:,0],       obstacles[:,1],       obstacles[:,2],       color="red",   s=3, label="Obstacles")
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.legend()
    plt.show()