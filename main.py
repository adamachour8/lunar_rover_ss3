"""
main.py — Mission lunaire SS3
-------------------------------
Point d'entrée de la mission autonome.

Pipeline complet :
  1. Génération / chargement du nuage de points LiDAR
  2. RANSAC   → séparation sol / terrain naturel / obstacles
  3. DBSCAN   → clustering des obstacles
  4. Filtration → classification objets d'intérêt vs obstacles
  5. Triangulation → NavMesh (grille navigable)
  6. A* + Orbite → planification mission complète
  7. Exécution physique :
       Pour chaque segment :
         a. SS3 envoie les waypoints à l'Arduino (série)
         b. L'Arduino exécute le mouvement et confirme "D"
         c. Avant chaque orbite : SS3 signal SS2 (TCP), attend "READY"
         d. Pendant l'orbite : SS2 prend 30 photos en autonome
         e. Après l'orbite  : SS3 attend confirmation "OK" de SS2
  8. Retour au point de départ

Flags de débogage :
  SIMULATION_MODE  = True   → pas de connexion Arduino ni SS2 (planning uniquement)
  AFFICHER_GRAPHES = True   → affiche les plots matplotlib
  EXPORTER_WAYPOINTS = True → sauvegarde mission_waypoints.txt
"""

import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import (
    NOM_FICHIER, RANSAC_THRESHOLD,
    DBSCAN_EPS, DBSCAN_MIN_SAMPLES,
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
from interfaces.motor_control      import connecter_arduino, executer_chemin

# ===========================================================================
# FLAGS — modifier ici pour passer du mode debug au mode mission réelle
# ===========================================================================

SIMULATION_MODE    = True   # True = pas d'Arduino ni SS2, planning uniquement
AFFICHER_GRAPHES   = True   # True = affiche les plots matplotlib après planification
EXPORTER_WAYPOINTS = True   # True = sauvegarde mission_waypoints.txt

# ===========================================================================
# 1. PERCEPTION
# ===========================================================================

print("=" * 60)
print("  Mission lunaire SS3 — Démarrage")
print("=" * 60)

print("\n[1/5] Chargement nuage de points...")
points_bruts = generer_terrain("simulation/" + NOM_FICHIER)
print(f"      {len(points_bruts)} points chargés")

print("[2/5] RANSAC — segmentation sol / obstacles...")
sol, terrain_naturel, obstacles_pts, _ = ransac(points_bruts, RANSAC_THRESHOLD)
print(f"      Sol : {len(sol)} pts | Terrain naturel : {len(terrain_naturel)} pts "
      f"| Obstacles : {len(obstacles_pts)} pts")

print("[3/5] DBSCAN — clustering obstacles...")
clusters = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
print(f"      {len(clusters)} clusters détectés")

print("[4/5] Filtration — classification objets d'intérêt / obstacles...")
objets_interet, obstacles = filtrer(clusters)
print(f"      Objets d'intérêt : {len(objets_interet)} | Obstacles : {len(obstacles)}")

for obj in objets_interet:
    print(f"      {obj}")
for obj in obstacles:
    print(f"      {obj}")

# ===========================================================================
# 2. NAVMESH + PLANIFICATION
# ===========================================================================

print("\n[5/5] NavMesh + Planification A*...")
points_navmesh = np.vstack([sol, terrain_naturel])
_, points_utilises, navigable, non_nav = perform_triangulation(points_navmesh)
print(f"      Triangles navigables : {len(navigable)} | Non-navigables : {len(non_nav)}")

grille, origine_xy, res = construire_grille(
    points_utilises, navigable, obstacles + objets_interet
)
print(f"      Grille : {grille.shape[0]}×{grille.shape[1]} cellules "
      f"({res}m/cell) — {(grille==0).sum()} cellules navigables")

# Planification complète
# envoyer_signal_ss2=False en simulation, True en mission réelle
chemins, waypoints_monde, ordre, stats = planifier_mission(
    grille, origine_xy, res,
    objets_interet,
    position_depart=(0.0, 0.0),
    envoyer_signal_ss2=not SIMULATION_MODE,
)

# ===========================================================================
# 3. RÉSUMÉ
# ===========================================================================

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

# ===========================================================================
# 4. EXÉCUTION PHYSIQUE (mission réelle uniquement)
# ===========================================================================

if not SIMULATION_MODE:

    if not objets_interet:
        print("⚠️  Aucun objet d'intérêt détecté — mission terminée sans déplacement")
        sys.exit(0)

    # Connexion Arduino
    arduino = connecter_arduino()
    if arduino is None:
        print("❌ Arduino non disponible — mission annulée")
        sys.exit(1)

    # Exécution du chemin complet
    # Note : les signaux SS2 sont déjà intégrés dans planifier_mission()
    # quand envoyer_signal_ss2=True. L'Arduino reçoit les coordonnées
    # converties en (distance, angle) par executer_chemin().

    coords = waypoints_pour_rover(waypoints_monde)
    print(f"🚀 Démarrage exécution — {len(coords)} waypoints\n")

    succes = executer_chemin(coords, arduino)

    print(f"\n{'='*60}")
    print(f"  Mission : {'✅ SUCCÈS' if succes else '❌ ÉCHEC'}")
    print(f"{'='*60}")

    arduino.close()
    print("🔌 Connexion Arduino fermée")

else:
    print("ℹ️  Mode simulation — aucun déplacement physique effectué")
    print("    Passer SIMULATION_MODE = False dans main.py pour la mission réelle")