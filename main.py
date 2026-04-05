import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial
import time
import numpy as np

from config import (
    NOM_FICHIER, RANSAC_THRESHOLD,
    DBSCAN_EPS, DBSCAN_MIN_SAMPLES, NOM_PORT,
)
from simulation.terrain_generator import generer_terrain
from perception.ransac             import ransac
from perception.DBSCAN             import dbscan
from perception.filtration         import filtrer
from navigation.triangulation      import perform_triangulation
from navigation.astar              import (
    construire_grille, planifier_mission,
    plot_astar, exporter_waypoints, waypoints_pour_rover
)
from interfaces.motor_control import executer_chemin

SIMULATION_MODE    = True
AFFICHER_GRAPHES   = True
EXPORTER_WAYPOINTS = True

print("=" * 60)
print("  Mission lunaire SS3 — Démarrage")
print("=" * 60)

print("\n[1/5] Chargement nuage de points...")
points_bruts = generer_terrain("simulation/" + NOM_FICHIER)
print(f"      {len(points_bruts)} points chargés")

print("[2/5] RANSAC — segmentation sol / obstacles...")
sol, terrain_naturel, obstacles_pts, _ = ransac(points_bruts, RANSAC_THRESHOLD)
print(f"      Sol : {len(sol)} pts | Terrain naturel : {len(terrain_naturel)} pts | Obstacles : {len(obstacles_pts)} pts")

print("[3/5] DBSCAN — clustering obstacles...")
clusters = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
print(f"      {len(clusters)} clusters détectés")

print("[4/5] Filtration — classification objets d'intérêt / obstacles...")
objets_interet, obstacles = filtrer(clusters)
print(f"      Objets d'intérêt : {len(objets_interet)} | Obstacles : {len(obstacles)}")

for obj in objets_interet + obstacles:
    print(f"      {obj}")

print("\n[5/5] NavMesh + Planification A*...")
points_navmesh = np.vstack([sol, terrain_naturel])
_, points_utilises, navigable, non_nav = perform_triangulation(points_navmesh)
print(f"      Triangles navigables : {len(navigable)} | Non-navigables : {len(non_nav)}")

grille, origine_xy, res = construire_grille(
    points_utilises, navigable, obstacles + objets_interet
)
print(f"      Grille : {grille.shape[0]}×{grille.shape[1]} cellules ({res}m/cell) — {(grille==0).sum()} cellules navigables")

chemins, waypoints_monde, ordre, stats = planifier_mission(
    grille, origine_xy, res,
    objets_interet,
    position_depart=(0.0, 0.0),
    envoyer_signal_ss2=not SIMULATION_MODE,
)

print(f"\n{'='*60}")
print(f"  RÉSUMÉ MISSION")
print(f"{'='*60}")
print(f"  Objets à visiter  : {len(ordre)}")
print(f"  Total waypoints   : {len(waypoints_monde)}")
print(f"  Distance totale   : {stats['distance_totale_m']:.2f} m")
print(f"  Vitesse rover     : {stats['vitesse_ms']} m/s")
print(f"  Temps estimé      : {stats['temps_str']}")
print(f"  Mode              : {'SIMULATION' if SIMULATION_MODE else 'MISSION RÉELLE'}")
print(f"{'='*60}\n")

if EXPORTER_WAYPOINTS:
    exporter_waypoints(waypoints_monde)

if AFFICHER_GRAPHES:
    plot_astar(grille, origine_xy, res,
               chemins, objets_interet, obstacles,
               waypoints_monde, ordre, position_depart=(0.0, 0.0))

if not SIMULATION_MODE:
    if not objets_interet:
        print("Aucun objet d'intérêt détecté — mission terminée")
        sys.exit(0)

    arduino = serial.Serial(NOM_PORT, baudrate=9600, timeout=5)
    print("Connexion Arduino...")
    time.sleep(2)
    arduino.reset_input_buffer()
    print("Prêt")

    coords = waypoints_pour_rover(waypoints_monde)
    print(f"Démarrage exécution — {len(coords)} waypoints\n")

    succes = executer_chemin(coords, arduino)

    print(f"\n{'='*60}")
    print(f"  Mission : {'SUCCÈS' if succes else 'ÉCHEC'}")
    print(f"{'='*60}")

    arduino.close()

else:
    print("Mode simulation — aucun déplacement physique effectué")
    print("Mettre SIMULATION_MODE = False pour la mission réelle")