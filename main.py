import os
import sys
NOM_PORT = "COM3" if sys.platform == "win32" else "/dev/ttyACM0"
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import psutil
import serial
import numpy as np

_perf_data = {}

def _log_perf(t0, etape):
    dt  = time.perf_counter() - t0
    mem = psutil.Process().memory_info().rss / 1024**2
    _perf_data[etape] = {"temps_s": round(dt, 2), "memoire_mb": round(mem, 1)}
    print(f"      >> Temps: {dt:.2f}s | Memoire: {mem:.1f} MB")
    return time.perf_counter()

from config import (
    NOM_FICHIER, RANSAC_THRESHOLD,
    DBSCAN_EPS, DBSCAN_MIN_SAMPLES, NOM_PORT,
    ARDUINO_BAUDRATE, ARDUINO_TIMEOUT, ORBIT_VITESSE_ROVER,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
from communication.envoyer_roche import envoyer_roche, attendre_fin_photo
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

SIMULATION_MODE    = False
AFFICHER_GRAPHES   = True
EXPORTER_WAYPOINTS = True

print("=" * 60)
print("  Mission lunaire SS3 — Démarrage")
print("=" * 60)

print("\n[1/5] Chargement nuage de points...")
t_total = time.perf_counter()
t0 = time.perf_counter()
points_bruts = generer_terrain(os.path.join(BASE_DIR, "simulation", NOM_FICHIER))
print(f"      {len(points_bruts)} points chargés")
t0 = _log_perf(t0, "chargement")

print("[2/5] RANSAC — segmentation sol / obstacles...")
sol, terrain_naturel, obstacles_pts, _ = ransac(points_bruts, RANSAC_THRESHOLD)
print(f"      Sol : {len(sol)} pts | Terrain naturel : {len(terrain_naturel)} pts | Obstacles : {len(obstacles_pts)} pts")
t0 = _log_perf(t0, "ransac")

print("[3/5] DBSCAN — clustering obstacles...")
clusters = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
print(f"      {len(clusters)} clusters détectés")
t0 = _log_perf(t0, "dbscan")

print("[4/5] Filtration — classification objets d'intérêt / obstacles...")
objets_interet, obstacles = filtrer(clusters)
print(f"      Objets d'intérêt : {len(objets_interet)} | Obstacles : {len(obstacles)}")

for obj in objets_interet + obstacles:
    print(f"      {obj}")
t0 = _log_perf(t0, "filtration")

print("\n[5/5] NavMesh + Planification A*...")
points_navmesh = np.vstack([sol, terrain_naturel])
_, points_utilises, navigable, non_nav = perform_triangulation(points_navmesh)
print(f"      Triangles navigables : {len(navigable)} | Non-navigables : {len(non_nav)}")

grille, origine_xy, res = construire_grille(
    points_utilises, navigable, obstacles, objets_interet=objets_interet
)

# Forcer la zone autour du départ navigable (angle mort du capteur LiDAR)
_rayon_depart = 0.30  # 30cm autour de (0,0)
_ix0 = int((0.0 - origine_xy[0]) / res)
_iy0 = int((0.0 - origine_xy[1]) / res)
_r_cells = int(np.ceil(_rayon_depart / res))
for _dx in range(-_r_cells, _r_cells + 1):
    for _dy in range(-_r_cells, _r_cells + 1):
        if np.sqrt(_dx**2 + _dy**2) * res <= _rayon_depart:
            _gx, _gy = _ix0 + _dx, _iy0 + _dy
            if 0 <= _gx < grille.shape[0] and 0 <= _gy < grille.shape[1]:
                if grille[_gx, _gy] != 1:
                    grille[_gx, _gy] = 0

# Dilater la zone navigable pour connecter les îlots isolés
from scipy import ndimage as _ndi
_nav_mask = (grille == 0)
_dilated  = _ndi.binary_dilation(_nav_mask, iterations=3)
grille    = np.where(_dilated & (grille != 1), 0, grille)

n_cellules_nav = int((grille == 0).sum())
print(f"      Grille : {grille.shape[0]}×{grille.shape[1]} cellules ({res}m/cell) — {n_cellules_nav} cellules navigables")
t0 = _log_perf(t0, "navmesh_grille")

chemins, waypoints_monde, ordre, stats = planifier_mission(
    grille, origine_xy, res,
    objets_interet,
    position_depart=(0.0, 0.0),
    envoyer_signal_ss2=False,  # SS2 appelé pendant l'exécution Arduino, pas ici
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
_log_perf(t0, "astar_planification")
temps_total = time.perf_counter() - t_total

# --- Rapport de performance ---
from config import (
    ASTAR_RAYON_ROVER, ORBIT_RADIUS, ORBIT_N_POINTS,
    DBSCAN_EPS, DBSCAN_MIN_SAMPLES, RANSAC_THRESHOLD, FILTRE_COMPACITE_MAX
)

orbit_completion = {}
for obj in ordre:
    n_orbit = len([wp for wp in waypoints_monde
                   if wp["type"] == "orbit" and f"Objet {obj.label}" in wp.get("label", "")])
    orbit_completion[obj.label] = {"segments": n_orbit, "total": ORBIT_N_POINTS,
                                   "taux": round(n_orbit / ORBIT_N_POINTS * 100, 1)}

rapport_path = os.path.join(BASE_DIR, "rapport_performance.txt")
with open(rapport_path, "w", encoding="utf-8") as f:
    f.write("=" * 60 + "\n")
    f.write("  RAPPORT DE PERFORMANCE — Mission lunaire SS3\n")
    f.write(f"  Fichier scan : {NOM_FICHIER}\n")
    f.write(f"  Date         : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write("=" * 60 + "\n\n")

    f.write("[ PERCEPTION ]\n")
    f.write(f"  Points bruts traites         : {len(points_bruts):,}\n")
    f.write(f"  Points sol (RANSAC)          : {len(sol):,}  ({len(sol)/len(points_bruts)*100:.1f}%)\n")
    f.write(f"  Points obstacles (RANSAC)    : {len(obstacles_pts):,}  ({len(obstacles_pts)/len(points_bruts)*100:.1f}%)\n")
    f.write(f"  Clusters detectes (DBSCAN)   : {len(clusters)}\n")
    f.write(f"  Objets d'interet classes     : {len(objets_interet)}\n")
    f.write(f"  Obstacles classes            : {len(obstacles)}\n\n")

    f.write("[ DETAILS OBJETS D'INTERET ]\n")
    for obj in objets_interet:
        cx, cy, cz = obj.centroide
        f.write(f"  Cluster {obj.label} | H: {obj.hauteur*100:.1f}cm | "
                f"L: {obj.longueur*100:.1f}cm | l: {obj.largeur*100:.1f}cm | "
                f"Dist: {obj.distance:.2f}m | "
                f"XYZ: ({cx:.3f}, {cy:.3f}, {cz:.3f})\n")
    f.write("\n")

    f.write("[ DETAILS OBSTACLES ]\n")
    for obj in obstacles:
        cx, cy, cz = obj.centroide
        f.write(f"  Cluster {obj.label} | H: {obj.hauteur*100:.1f}cm | "
                f"L: {obj.longueur*100:.1f}cm | l: {obj.largeur*100:.1f}cm | "
                f"Dist: {obj.distance:.2f}m | "
                f"XYZ: ({cx:.3f}, {cy:.3f}, {cz:.3f})\n")
    f.write("\n")

    f.write("[ NAVIGATION ]\n")
    f.write(f"  Grille                       : {grille.shape[0]}x{grille.shape[1]} cellules ({res}m/cell)\n")
    f.write(f"  Cellules navigables          : {n_cellules_nav}\n")
    f.write(f"  Objets visites               : {len(ordre)}\n")
    f.write(f"  Total waypoints              : {len(waypoints_monde)}\n")
    f.write(f"  Distance totale planifiee    : {stats['distance_totale_m']:.2f} m\n")
    f.write(f"  Temps mission estime         : {stats['temps_str']}\n")
    f.write(f"  Vitesse rover                : {stats['vitesse_ms']} m/s\n")
    f.write(f"  Rayon securite obstacles     : {ASTAR_RAYON_ROVER*100:.0f} cm\n")
    f.write(f"  Rayon orbite objets          : {ORBIT_RADIUS*100:.0f} cm\n\n")

    f.write("[ COMPLETION DES ORBITES ]\n")
    for label, info in orbit_completion.items():
        f.write(f"  Objet {label} : {info['segments']}/{info['total']} segments ({info['taux']}%)\n")
    f.write("\n")

    f.write("[ TEMPS D'EXECUTION PAR ETAPE ]\n")
    noms = {
        "chargement":        "Chargement nuage de points",
        "ransac":            "RANSAC segmentation",
        "dbscan":            "DBSCAN clustering",
        "filtration":        "Filtration classification",
        "navmesh_grille":    "NavMesh + Grille A*",
        "astar_planification": "Planification A*",
    }
    for cle, nom in noms.items():
        if cle in _perf_data:
            f.write(f"  {nom:<30} : {_perf_data[cle]['temps_s']:.2f}s\n")
    f.write(f"  {'Temps total pipeline':<30} : {temps_total:.2f}s\n\n")

    f.write("[ MEMOIRE PAR ETAPE ]\n")
    for cle, nom in noms.items():
        if cle in _perf_data:
            f.write(f"  {nom:<30} : {_perf_data[cle]['memoire_mb']:.1f} MB\n")
    f.write("\n")

    f.write("[ PARAMETRES UTILISES ]\n")
    f.write(f"  RANSAC_THRESHOLD             : {RANSAC_THRESHOLD}\n")
    f.write(f"  DBSCAN_EPS                   : {DBSCAN_EPS}\n")
    f.write(f"  DBSCAN_MIN_SAMPLES           : {DBSCAN_MIN_SAMPLES}\n")
    f.write(f"  FILTRE_COMPACITE_MAX         : {FILTRE_COMPACITE_MAX}\n")
    f.write(f"  ORBIT_N_POINTS               : {ORBIT_N_POINTS}\n")
    f.write("=" * 60 + "\n")

print(f"[Rapport] Performance sauvegardee -> rapport_performance.txt")

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

    arduino = serial.Serial(NOM_PORT, baudrate=ARDUINO_BAUDRATE, timeout=ARDUINO_TIMEOUT)
    print("Connexion Arduino...")
    time.sleep(2)
    arduino.reset_input_buffer()

    arduino.write(b"PING\n")
    reponse_ping = arduino.readline().decode('utf-8').strip()
    print(f"Connexion ELEGOO : {reponse_ping}")
    if reponse_ping != "PONG":
        print("ELEGOO non disponible — vérifier la connexion USB")
        arduino.close()
        sys.exit(1)
    print("Prêt\n")

    pos_courante   = (0.0, 0.0)
    succes_global  = True

    for obj in ordre:
        label = obj.label

        approche_wps = [wp for wp in waypoints_monde
                        if wp["type"] == "travel" and f"Objet {label}" in wp.get("label", "")]
        orbite_wps   = [wp for wp in waypoints_monde
                        if wp["type"] == "orbit"  and f"Objet {label}" in wp.get("label", "")]

        # 1. Déplacement vers le point d'entrée de l'orbite
        if approche_wps:
            coords = [pos_courante] + [(wp["x"], wp["y"]) for wp in approche_wps]
            print(f"→ Approche Objet {label} ({len(coords)-1} waypoints)")
            if not executer_chemin(coords, arduino):
                print(f"Échec approche Objet {label} — mission arrêtée")
                succes_global = False
                break
            pos_courante = (approche_wps[-1]["x"], approche_wps[-1]["y"])

        # 2. Rover devant la roche — signal SS2
        if len(orbite_wps) > 1:
            duree_orbite_s = sum(
                np.linalg.norm(np.array([orbite_wps[i+1]["x"] - orbite_wps[i]["x"],
                                         orbite_wps[i+1]["y"] - orbite_wps[i]["y"]]))
                for i in range(len(orbite_wps) - 1)
            ) / ORBIT_VITESSE_ROVER
        else:
            duree_orbite_s = (2 * np.pi * 0.75) / ORBIT_VITESSE_ROVER
        envoyer_roche(obj, pos_courante, duree_orbite_s)

        # 3. Orbite autour de la roche
        if orbite_wps:
            coords = [pos_courante] + [(wp["x"], wp["y"]) for wp in orbite_wps]
            print(f"→ Orbite Objet {label} ({len(coords)-1} waypoints)")
            executer_chemin(coords, arduino)
            pos_courante = (orbite_wps[-1]["x"], orbite_wps[-1]["y"])

        # 4. Attendre confirmation fin de photos SS2
        attendre_fin_photo(label)

    # 5. Retour au point de départ
    retour_wps = [wp for wp in waypoints_monde if wp["type"] == "return"]
    if retour_wps and succes_global:
        coords = [pos_courante] + [(wp["x"], wp["y"]) for wp in retour_wps]
        print(f"→ Retour départ ({len(coords)-1} waypoints)")
        executer_chemin(coords, arduino)

    print(f"\n{'='*60}")
    print(f"  Mission : {'SUCCÈS' if succes_global else 'ÉCHEC'}")
    print(f"{'='*60}")

    arduino.close()

else:
    print("Mode simulation — aucun déplacement physique effectué")
    print("Mettre SIMULATION_MODE = False pour la mission réelle")