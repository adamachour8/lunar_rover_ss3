import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from config import NOM_FICHIER, RANSAC_THRESHOLD, DBSCAN_EPS, DBSCAN_MIN_SAMPLES

from simulation.terrain_generator import generer_terrain
from perception.ransac            import ransac
from perception.DBSCAN            import dbscan
from perception.filtration        import filtrer
from navigation.triangulation     import perform_triangulation
from navigation.astar             import construire_grille, planifier_mission, plot_astar

# PIPELINE ------------------------------------------------------------------

points_bruts                            = generer_terrain("simulation/" + NOM_FICHIER)
sol, terrain_naturel, obstacles_pts, _  = ransac(points_bruts, RANSAC_THRESHOLD)
clusters                                = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
objets_interet, obstacles               = filtrer(clusters)

# Résultats perception
for obj in objets_interet:
    print(obj)
for obj in obstacles:
    print(obj)

# NavMesh
points_navmesh                           = np.vstack([sol, terrain_naturel])
_, points_utilises, navigable, non_nav   = perform_triangulation(points_navmesh)

print(f"\nNavMesh — Triangles navigables     : {len(navigable)}")
print(f"NavMesh — Triangles non-navigables : {len(non_nav)}")

# Pathfinding A*
grille, origine_xy, res = construire_grille(
    points_utilises, navigable, obstacles + objets_interet
)

chemins, waypoints, ordre = planifier_mission(
    grille, origine_xy, res,
    objets_interet, position_depart=(0.0, 0.0)
)

plot_astar(grille, origine_xy, res,
           chemins, objets_interet, obstacles,
           waypoints, ordre, position_depart=(0.0, 0.0))