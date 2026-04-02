import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import time
import serial
from config import NOM_FICHIER, RANSAC_THRESHOLD, DBSCAN_EPS, DBSCAN_MIN_SAMPLES, VITESSE_MS_PAR_METRE, VITESSE_MS_PAR_DEGRE

from simulation.terrain_generator import generer_terrain
from perception.ransac            import ransac
from perception.DBSCAN            import dbscan
from perception.filtration        import filtrer
from navigation.triangulation     import perform_triangulation
from navigation.astar             import construire_grille, planifier_mission, plot_astar
from interfaces.motor_control     import executer_chemin

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

# print(chemins)
# print(waypoints)

# plot_astar(grille, origine_xy, res,
#            chemins, objets_interet, obstacles,
#            waypoints, ordre, position_depart=(0.0, 0.0))


# --- À partir d'ici: boucle de chacun des chemins (1 à la fois, vu qu'on va devoir appeler C2 pour la photogrammétrie) ---


arduino = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=2)
time.sleep(2)

#executer_chemin(chemin, arduino, VITESSE_MS_PAR_METRE, VITESSE_MS_PAR_DEGRE)