"""
navigation/astar.py
-------------------
Pathfinding A* sur le NavMesh triangulé — Mission lunaire SS3.

Pipeline complet :
  1. Construction grille 2D depuis triangles navigables
  2. Inflation des obstacles (rayon de sécurité >= rayon du rover)
  3. A* pour trouver le chemin entre deux points
  4. Ordre de visite glouton (toujours l'objet le plus proche)
  5. Orbite autour de chaque objet d'intérêt (rayon ORBIT_RADIUS, ~10 cm)
  6. Retour à l'origine (position_depart)

Sorties :
  - chemins        : liste de chemins bruts (cellules grille) — usage interne
  - waypoints_monde: liste de dicts {x, y, type} en mètres — pour le Rover / Mission Planner
  - ordre          : liste d'ObjetDetecte dans l'ordre de visite
"""

import numpy as np
import heapq
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (
    ASTAR_RESOLUTION,
    ASTAR_RAYON_ROVER,
    ASTAR_RAYON_INFLATION,
    ASTAR_SCAN_DISTANCE,
    ORBIT_RADIUS,
    ORBIT_N_POINTS,
    ORBIT_VITESSE_ROVER,
)


# ===========================================================================
# 1. CONSTRUCTION DE LA GRILLE 2D
# ===========================================================================

def construire_grille(points_navmesh, navigable, tous_les_objets,
                      resolution=ASTAR_RESOLUTION,
                      rayon_inflation=ASTAR_RAYON_INFLATION):
    """
    Construit une grille 2D occupancy map depuis le NavMesh.

    Args:
        points_navmesh  : np.array (N, 3) — points du sol triangulé
        navigable       : np.array — simplices navigables (output triangulation)
        tous_les_objets : liste d'ObjetDetecte — obstacles + objets d'intérêt
        resolution      : taille d'une cellule en mètres
        rayon_inflation : rayon de sécurité >= ASTAR_RAYON_ROVER

    Returns:
        grille     : np.array 2D — 0=navigable, 1=obstacle, 2=hors-carte
        origine_xy : tuple (x_min, y_min)  ← coin bas-gauche du terrain réel
        resolution : float
    """
    assert rayon_inflation >= ASTAR_RAYON_ROVER, \
        f"rayon_inflation ({rayon_inflation}) doit être >= rayon du rover ({ASTAR_RAYON_ROVER})"

    x_min = points_navmesh[:, 0].min() - 0.5
    x_max = points_navmesh[:, 0].max() + 0.5
    y_min = points_navmesh[:, 1].min() - 0.5
    y_max = points_navmesh[:, 1].max() + 0.5

    nx = int(np.ceil((x_max - x_min) / resolution))
    ny = int(np.ceil((y_max - y_min) / resolution))

    grille = np.full((nx, ny), 2, dtype=np.int8)  # 2 = hors-carte

    # Marquer les triangles navigables comme libres (0)
    for simplex in navigable:
        pts = points_navmesh[simplex][:, :2]
        ix  = ((pts[:, 0] - x_min) / resolution).astype(int)
        iy  = ((pts[:, 1] - y_min) / resolution).astype(int)
        for gx in range(max(0, ix.min()), min(nx - 1, ix.max()) + 1):
            for gy in range(max(0, iy.min()), min(ny - 1, iy.max()) + 1):
                cx = x_min + (gx + 0.5) * resolution
                cy = y_min + (gy + 0.5) * resolution
                if _point_dans_triangle(np.array([cx, cy]), pts):
                    grille[gx, gy] = 0

    # Gonfler les obstacles (garantit que le rover fit dans les couloirs)
    cellules_inflation = int(np.ceil(rayon_inflation / resolution))
    for obj in tous_les_objets:
        ix_obj = int((obj.centroide[0] - x_min) / resolution)
        iy_obj = int((obj.centroide[1] - y_min) / resolution)
        for dx in range(-cellules_inflation, cellules_inflation + 1):
            for dy in range(-cellules_inflation, cellules_inflation + 1):
                if np.sqrt(dx**2 + dy**2) * resolution <= rayon_inflation:
                    gx, gy = ix_obj + dx, iy_obj + dy
                    if 0 <= gx < nx and 0 <= gy < ny:
                        grille[gx, gy] = 1

    # Boucher les trous hors-carte isolés (artefacts de scan LiDAR)
    grille = _boucher_trous_horscarte(grille)

    return grille, (x_min, y_min), resolution


def _boucher_trous_horscarte(grille, taille_max_ilot=10):
    """
    Remplace les îlots hors-carte (valeur 2) isolés par des cellules navigables (0).

    Un îlot est considéré isolé si sa taille (en cellules connectées) est <= taille_max_ilot
    ET qu'il est entouré uniquement de cellules navigables (pas de bord de carte).
    Ces îlots sont des artefacts de scan LiDAR — trous dans le nuage de points.

    Args:
        grille         : np.array 2D — 0=navigable, 1=obstacle, 2=hors-carte
        taille_max_ilot: Taille max (cellules) d'un îlot pour être considéré artefact.
                         Ajuster selon la densité du scan. Défaut=10 (~10x10cm à 0.1m/cell)

    Returns:
        grille : np.array 2D modifiée (in-place copy)
    """
    grille = grille.copy()
    nx, ny = grille.shape
    visite = np.zeros((nx, ny), dtype=bool)

    for ix in range(nx):
        for iy in range(ny):
            if grille[ix, iy] != 2 or visite[ix, iy]:
                continue

            # BFS pour trouver l'îlot complet de cellules hors-carte connectées
            ilot        = []
            file        = [(ix, iy)]
            visite[ix, iy] = True
            touche_bord = False

            while file:
                cx, cy = file.pop()
                ilot.append((cx, cy))

                # Si on touche le bord de la grille, c'est le vrai bord — ne pas boucher
                if cx == 0 or cx == nx - 1 or cy == 0 or cy == ny - 1:
                    touche_bord = True

                for ddx, ddy in [(-1,0),(1,0),(0,-1),(0,1),
                                  (-1,-1),(-1,1),(1,-1),(1,1)]:
                    nx2, ny2 = cx + ddx, cy + ddy
                    if 0 <= nx2 < nx and 0 <= ny2 < ny and not visite[nx2, ny2]:
                        if grille[nx2, ny2] == 2:
                            visite[nx2, ny2] = True
                            file.append((nx2, ny2))

            # Boucher seulement si : petit îlot ET n'est pas le bord extérieur
            if not touche_bord and len(ilot) <= taille_max_ilot:
                for cx, cy in ilot:
                    grille[cx, cy] = 0

    n_bouches = int((grille == 0).sum())
    return grille


def _point_dans_triangle(p, triangle):
    """Test si le point p est dans le triangle (3 sommets 2D)."""
    a, b, c   = triangle
    v0, v1, v2 = c - a, b - a, p - a
    d00, d01, d02 = np.dot(v0, v0), np.dot(v0, v1), np.dot(v0, v2)
    d11, d12      = np.dot(v1, v1), np.dot(v1, v2)
    denom = d00 * d11 - d01 * d01
    if abs(denom) < 1e-10:
        return False
    inv = 1.0 / denom
    u   = (d11 * d02 - d01 * d12) * inv
    v   = (d00 * d12 - d01 * d02) * inv
    return (u >= 0) and (v >= 0) and (u + v <= 1)


# ===========================================================================
# 2. ALGORITHME A*
# ===========================================================================

def astar(grille, debut, fin):
    """
    A* sur grille 2D avec 8-connectivité.

    Args:
        grille : np.array 2D — 0=libre, 1=obstacle, 2=hors-carte
        debut  : tuple (ix, iy)
        fin    : tuple (ix, iy)

    Returns:
        chemin : liste de tuples (ix, iy), ou None si pas de chemin
    """
    nx, ny = grille.shape

    def h(a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    open_set  = [(0, debut)]
    came_from = {}
    g_score   = {debut: 0}

    while open_set:
        _, cur = heapq.heappop(open_set)

        if cur == fin:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(debut)
            return path[::-1]

        x, y = cur
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                gx, gy = x + dx, y + dy
                if 0 <= gx < nx and 0 <= gy < ny and grille[gx, gy] == 0:
                    voisin      = (gx, gy)
                    g_tentative = g_score[cur] + np.sqrt(dx**2 + dy**2)
                    if g_tentative < g_score.get(voisin, float('inf')):
                        came_from[voisin] = cur
                        g_score[voisin]   = g_tentative
                        f = g_tentative + h(voisin, fin)
                        heapq.heappush(open_set, (f, voisin))

    return None


# ===========================================================================
# 3. UTILITAIRES COORDONNÉES
# ===========================================================================

def monde_vers_grille(x, y, origine_xy, resolution):
    """Convertit des coordonnées monde (m) en indices grille."""
    return (int((x - origine_xy[0]) / resolution),
            int((y - origine_xy[1]) / resolution))

def grille_vers_monde(ix, iy, origine_xy, resolution):
    """Convertit des indices grille en coordonnées monde (m) — centre de cellule."""
    return (origine_xy[0] + (ix + 0.5) * resolution,
            origine_xy[1] + (iy + 0.5) * resolution)

def chemin_vers_monde(chemin, origine_xy, resolution):
    """Convertit une liste de cellules grille en liste de (x, y) monde (m)."""
    return [grille_vers_monde(ix, iy, origine_xy, resolution) for ix, iy in chemin]

def cellule_libre_proche(grille, ix, iy):
    """Trouve la cellule libre la plus proche de (ix, iy)."""
    nx, ny = grille.shape
    if 0 <= ix < nx and 0 <= iy < ny and grille[ix, iy] == 0:
        return ix, iy
    for rayon in range(1, 30):
        for dx in range(-rayon, rayon + 1):
            for dy in range(-rayon, rayon + 1):
                gx, gy = ix + dx, iy + dy
                if 0 <= gx < nx and 0 <= gy < ny and grille[gx, gy] == 0:
                    return gx, gy
    return None


# ===========================================================================
# 4. ORBITE AUTOUR D'UN OBJET D'INTÉRÊT — TOUR COMPLET PAR A*
# ===========================================================================

def generer_points_orbite(centroide_xy, rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS):
    """
    Génère n_points régulièrement espacés sur un cercle autour du centroïde.

    Returns:
        points : list of (x, y) en mètres, dans l'ordre trigonométrique (CCW)
    """
    cx, cy = centroide_xy
    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    return [(cx + rayon * np.cos(a), cy + rayon * np.sin(a)) for a in angles]


def orbite_complete_astar(grille, origine_xy, resolution,
                          centroide_xy, pos_rover_xy,
                          rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS):
    """
    Planifie un tour COMPLET autour d'un objet en utilisant A* entre chaque
    point consécutif du cercle d'orbite.

    Stratégie :
      1. Générer n_points sur le cercle (sens CCW)
      2. Trouver le point d'entrée le plus proche du rover
      3. Réordonner depuis ce point d'entrée
      4. Pour chaque paire de points consécutifs : A* pour trouver le chemin
         navigable entre eux — garantit que le rover longe vraiment l'objet
      5. Bouclage : retour au point d'entrée pour fermer le cercle

    Returns:
        chemins_orbite : liste de chemins A* (cellules grille) — un par segment
        pts_orbite_ordonnes : liste de (x,y) monde dans l'ordre de visite
        n_segments_ok : nombre de segments réussis (pour stats)
    """
    candidats = generer_points_orbite(centroide_xy, rayon, n_points)

    # Identifier les points dont la cellule est navigable ou très proche d'une cellule libre
    pts_valides = []
    for pt in candidats:
        ix, iy = monde_vers_grille(*pt, origine_xy, resolution)
        cell = cellule_libre_proche(grille, ix, iy)
        if cell is not None:
            dist_snap = np.linalg.norm(
                np.array(grille_vers_monde(*cell, origine_xy, resolution)) - np.array(pt)
            )
            if dist_snap <= resolution * 4:   # tolérance : 4 cellules max
                pts_valides.append(pt)

    if len(pts_valides) < 2:
        print(f"  [Orbite] Pas assez de points navigables ({len(pts_valides)}) — orbite impossible")
        return [], [], 0

    # Trouver le point d'entrée le plus proche du rover parmi les valides
    dists = [np.linalg.norm(np.array(pt) - np.array(pos_rover_xy)) for pt in pts_valides]
    idx_entree = int(np.argmin(dists))

    # Réordonner depuis le point d'entrée (sens CCW conservé)
    pts_ordonnes = pts_valides[idx_entree:] + pts_valides[:idx_entree]

    # Fermer la boucle : revenir au point de départ de l'orbite
    pts_ordonnes_boucle = pts_ordonnes + [pts_ordonnes[0]]

    chemins_orbite = []
    pts_confirmes  = [pts_ordonnes_boucle[0]]
    n_ok           = 0

    for i in range(len(pts_ordonnes_boucle) - 1):
        pt_a = pts_ordonnes_boucle[i]
        pt_b = pts_ordonnes_boucle[i + 1]

        cell_a = cellule_libre_proche(grille, *monde_vers_grille(*pt_a, origine_xy, resolution))
        cell_b = cellule_libre_proche(grille, *monde_vers_grille(*pt_b, origine_xy, resolution))

        if cell_a is None or cell_b is None:
            continue  # segment impossible, on saute

        if cell_a == cell_b:
            # Même cellule — pas besoin de A*, on note juste le point
            pts_confirmes.append(pt_b)
            n_ok += 1
            continue

        ch = astar(grille, cell_a, cell_b)
        if ch:
            chemins_orbite.append(ch)
            # Ajouter les waypoints monde de ce segment (sans le premier, déjà dans liste)
            for wx, wy in chemin_vers_monde(ch, origine_xy, resolution)[1:]:
                pts_confirmes.append((wx, wy))
            n_ok += 1
        # Si A* échoue sur un segment, on continue avec le suivant (dégradé gracieux)

    return chemins_orbite, pts_confirmes, n_ok


# ===========================================================================
# 5. ORDRE DE VISITE GLOUTON
# ===========================================================================

def ordre_visite_glouton(position_depart, objets):
    """
    Nearest-neighbor TSP : toujours visiter l'objet d'intérêt le plus proche.
    """
    restants = list(objets)
    ordre    = []
    pos      = np.array(position_depart)
    while restants:
        idx = np.argmin([np.linalg.norm(pos - o.centroide[:2]) for o in restants])
        ordre.append(restants.pop(idx))
        pos = ordre[-1].centroide[:2]
    return ordre


# ===========================================================================
# 6. CALCUL DE DISTANCE ET TEMPS
# ===========================================================================

def calculer_stats_mission(waypoints_monde, vitesse=ORBIT_VITESSE_ROVER):
    """
    Calcule la distance totale parcourue et le temps estimé de la mission.

    Args:
        waypoints_monde : liste de dicts {x, y, ...}
        vitesse         : vitesse du rover en m/s

    Returns:
        dict {
            distance_totale_m : float — distance en mètres
            temps_s           : float — temps en secondes
            temps_str         : str   — format lisible "Xmin Ys"
        }
    """
    dist = 0.0
    for i in range(1, len(waypoints_monde)):
        dx = waypoints_monde[i]["x"] - waypoints_monde[i-1]["x"]
        dy = waypoints_monde[i]["y"] - waypoints_monde[i-1]["y"]
        dist += np.sqrt(dx**2 + dy**2)

    temps_s = dist / vitesse if vitesse > 0 else 0.0
    minutes = int(temps_s // 60)
    secondes = temps_s % 60

    if minutes > 0:
        temps_str = f"{minutes}min {secondes:.1f}s"
    else:
        temps_str = f"{secondes:.1f}s"

    return {
        "distance_totale_m": round(dist, 3),
        "temps_s":           round(temps_s, 1),
        "temps_str":         temps_str,
        "vitesse_ms":        vitesse,
    }


# ===========================================================================
# 7. PLANIFICATION COMPLÈTE
# ===========================================================================

def planifier_mission(grille, origine_xy, resolution,
                      objets_interet, position_depart=(0.0, 0.0)):
    """
    Planifie la mission complète :
        Départ → approche obj1 → TOUR COMPLET obj1 → approche obj2 → ... → retour

    Returns:
        chemins         : liste de chemins bruts (cellules grille) — usage interne / debug
        waypoints_monde : liste de dicts {x (m), y (m), type (str), label (str)}
                          → prêt pour executer_chemin() et Mission Planner
        ordre           : liste d'ObjetDetecte dans l'ordre de visite
        stats           : dict distance + temps de mission
    """
    ordre           = ordre_visite_glouton(position_depart, objets_interet)
    chemins         = []
    waypoints_monde = []

    # Point de départ
    waypoints_monde.append({
        "x": float(position_depart[0]),
        "y": float(position_depart[1]),
        "type": "depart",
        "label": "Départ"
    })

    pos = position_depart

    for obj in ordre:
        cx, cy = float(obj.centroide[0]), float(obj.centroide[1])

        # ── Étape A : Approche vers le point d'orbite le plus proche ──────
        pts_cercle     = generer_points_orbite((cx, cy), ORBIT_RADIUS, ORBIT_N_POINTS)
        dists_approche = [np.linalg.norm(np.array(pt) - np.array(pos)) for pt in pts_cercle]
        pt_entree      = pts_cercle[int(np.argmin(dists_approche))]

        d_cell = cellule_libre_proche(grille, *monde_vers_grille(*pos,       origine_xy, resolution))
        f_cell = cellule_libre_proche(grille, *monde_vers_grille(*pt_entree, origine_xy, resolution))

        if d_cell is None or f_cell is None:
            print(f"[A*] Objet {obj.label} — cellule d'approche introuvable, objet ignoré")
            continue

        ch_approche = astar(grille, d_cell, f_cell)
        if ch_approche:
            chemins.append(ch_approche)
            for wx, wy in chemin_vers_monde(ch_approche, origine_xy, resolution)[1:]:
                waypoints_monde.append({
                    "x": round(wx, 4), "y": round(wy, 4),
                    "type": "travel",
                    "label": f"Approche Objet {obj.label}"
                })
            pos = pt_entree
            print(f"[A*] Approche Objet {obj.label} ({obj.categorie}) ✓")
        else:
            print(f"[A*] Objet {obj.label} — aucun chemin d'approche, objet ignoré")
            continue

        # ── Étape B : Tour COMPLET autour de l'objet via A* ───────────────
        chemins_orbite, pts_orbite, n_ok = orbite_complete_astar(
            grille, origine_xy, resolution,
            (cx, cy), pos,
            rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS
        )

        if chemins_orbite:
            chemins.extend(chemins_orbite)
            for wx, wy in pts_orbite:
                waypoints_monde.append({
                    "x": round(float(wx), 4), "y": round(float(wy), 4),
                    "type": "orbit",
                    "label": f"Orbite Objet {obj.label} (r={ORBIT_RADIUS*100:.0f}cm)"
                })
            pos = pts_orbite[-1]
            print(f"[A*] Tour Objet {obj.label} ✓ — {n_ok}/{ORBIT_N_POINTS} segments, "
                  f"{len(pts_orbite)} waypoints")
        else:
            # Fallback : scan simple si l'orbite est bloquée de tous côtés
            print(f"[A*] Objet {obj.label} — orbite impossible, scan simple (fallback)")
            direction = np.array([cx, cy]) - np.array(pos)
            dist      = np.linalg.norm(direction)
            if dist > ASTAR_SCAN_DISTANCE:
                pt_scan = np.array(pos) + (direction / dist) * (dist - ASTAR_SCAN_DISTANCE)
                waypoints_monde.append({
                    "x": round(float(pt_scan[0]), 4), "y": round(float(pt_scan[1]), 4),
                    "type": "scan",
                    "label": f"Scan Objet {obj.label} (fallback)"
                })

    # ── Étape finale : Retour à l'origine ─────────────────────────────────
    d_cell = cellule_libre_proche(grille, *monde_vers_grille(*pos,             origine_xy, resolution))
    f_cell = cellule_libre_proche(grille, *monde_vers_grille(*position_depart, origine_xy, resolution))

    if d_cell and f_cell:
        ch_retour = astar(grille, d_cell, f_cell)
        if ch_retour:
            chemins.append(ch_retour)
            for wx, wy in chemin_vers_monde(ch_retour, origine_xy, resolution)[1:]:
                waypoints_monde.append({
                    "x": round(wx, 4), "y": round(wy, 4),
                    "type": "return",
                    "label": "Retour départ"
                })
            print(f"[A*] Retour origine ✓")
        else:
            print("[A*] Retour origine — aucun chemin trouvé")
    else:
        print("[A*] Retour origine — cellule introuvable")

    waypoints_monde.append({
        "x": float(position_depart[0]),
        "y": float(position_depart[1]),
        "type": "arrivee",
        "label": "Arrivée"
    })

    stats = calculer_stats_mission(waypoints_monde, vitesse=ORBIT_VITESSE_ROVER)
    return chemins, waypoints_monde, ordre, stats


# ===========================================================================
# 8. EXPORT WAYPOINTS POUR ROVER / MISSION PLANNER
# ===========================================================================

def exporter_waypoints(waypoints_monde, fichier="mission_waypoints.txt"):
    """
    Exporte les waypoints en coordonnées réelles (mètres) dans un fichier texte.
    Format compatible Mission Planner / ArduPilot (simplifié) et executer_chemin().

    Colonnes : index | x (m) | y (m) | type | label
    """
    with open(fichier, "w", encoding="utf-8") as f:
        f.write("# Mission lunaire SS3 — Waypoints en coordonnées réelles (mètres)\n")
        f.write("# index\tx_m\ty_m\ttype\tlabel\n")
        for i, wp in enumerate(waypoints_monde):
            f.write(f"{i}\t{wp['x']:.4f}\t{wp['y']:.4f}\t{wp['type']}\t{wp['label']}\n")
    print(f"[Export] {len(waypoints_monde)} waypoints sauvegardés → {fichier}")


def waypoints_pour_rover(waypoints_monde):
    """
    Retourne une liste de tuples (x, y) en mètres, prêts pour executer_chemin().
    Filtre les doublons consécutifs et les points trop proches (< 1 cm).

    Returns:
        liste de (x, y) float — coordonnées réelles en mètres
    """
    coords = [(wp["x"], wp["y"]) for wp in waypoints_monde]

    # Dédoublonner les points consécutifs trop proches
    seuil = 0.01  # 1 cm
    filtres = [coords[0]]
    for pt in coords[1:]:
        if np.linalg.norm(np.array(pt) - np.array(filtres[-1])) > seuil:
            filtres.append(pt)

    return filtres


# ===========================================================================
# 9. VISUALISATION (optionnelle — debug local uniquement, pas de sauvegarde PNG)
# ===========================================================================

def plot_astar(grille, origine_xy, resolution,
               chemins, objets_interet, obstacles,
               waypoints_monde, ordre, position_depart=(0.0, 0.0)):
    """
    Vue 2D du dessus — NavMesh + chemins A* — affichage écran uniquement.
    Aucune sauvegarde de fichier PNG.
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.colors import ListedColormap
    except ImportError:
        print("[plot] matplotlib non disponible — visualisation ignorée")
        return

    COULEURS = ['#E63946', '#2196F3', '#FF9800', '#9C27B0', '#00BCD4', '#8BC34A']

    fig, ax = plt.subplots(figsize=(13, 11))
    nx, ny  = grille.shape
    extent  = [origine_xy[0], origine_xy[0] + nx * resolution,
               origine_xy[1], origine_xy[1] + ny * resolution]

    cmap = ListedColormap(['#C8F5C8', '#FFCDD2', '#F0F0F0'])
    ax.imshow(grille.T, origin='lower', extent=extent,
              cmap=cmap, vmin=0, vmax=2, alpha=0.8, interpolation='nearest')

    # Chemins A*
    for i, chemin in enumerate(chemins):
        c  = COULEURS[i % len(COULEURS)]
        lb = (f"Segment {i+1} → Objet {ordre[i].label}"
              if i < len(ordre) else "Retour origine")
        xs = [grille_vers_monde(p[0], p[1], origine_xy, resolution)[0] for p in chemin]
        ys = [grille_vers_monde(p[0], p[1], origine_xy, resolution)[1] for p in chemin]
        ax.plot(xs, ys, color=c, linewidth=2.5, zorder=4, label=lb)
        step = max(1, len(xs) // 8)
        for j in range(0, len(xs) - 1, step):
            ax.annotate('', xy=(xs[j+1], ys[j+1]), xytext=(xs[j], ys[j]),
                        arrowprops=dict(arrowstyle='->', color=c, lw=1.5))

    # Points d'orbite
    orbit_wps = [wp for wp in waypoints_monde if wp["type"] == "orbit"]
    if orbit_wps:
        ox = [wp["x"] for wp in orbit_wps]
        oy = [wp["y"] for wp in orbit_wps]
        ax.scatter(ox, oy, s=30, color='#FF6F00', zorder=5, marker='o',
                   label=f"Orbite (r={ORBIT_RADIUS*100:.0f}cm)", alpha=0.7)

    # Objets d'intérêt
    for obj in objets_interet:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=250, color='#1565C0', zorder=6, marker='*',
                   edgecolors='white', linewidth=0.5)
        circle = plt.Circle((obj.centroide[0], obj.centroide[1]),
                             ORBIT_RADIUS, color='#1565C0', fill=False,
                             linestyle='--', linewidth=1, alpha=0.5)
        ax.add_patch(circle)
        ax.annotate(f" Objet {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8.5, color='#0D47A1',
                    bbox=dict(boxstyle='round,pad=0.3', fc='white',
                              ec='#1565C0', alpha=0.85))

    # Obstacles
    for obj in obstacles:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=200, color='#B71C1C', zorder=6, marker='X',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Obstacle {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8, color='#B71C1C',
                    bbox=dict(boxstyle='round,pad=0.3', fc='white',
                              ec='#B71C1C', alpha=0.85))

    # Départ
    ax.scatter(*position_depart, s=350, color='#FF6F00',
               zorder=7, marker='D', edgecolors='white', linewidth=1)
    ax.annotate(f' DÉPART\n ({position_depart[0]:.2f}, {position_depart[1]:.2f})',
                position_depart, fontsize=9, fontweight='bold', color='#E65100',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#FF6F00', alpha=0.9))

    # Légende
    handles = [
        mpatches.Patch(color='#C8F5C8', label='Zone navigable'),
        mpatches.Patch(color='#FFCDD2', label='Obstacle (zone gonflée ≥ rayon rover)'),
        mpatches.Patch(color='#F0F0F0', label='Hors-carte'),
        plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='#1565C0',
                   markersize=13, label="Objet d'intérêt"),
        plt.Line2D([0], [0], marker='X', color='w', markerfacecolor='#B71C1C',
                   markersize=11, label='Obstacle détecté'),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='#FF6F00',
                   markersize=11, label='Départ / Arrivée'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#FF6F00',
                   markersize=8, label=f"Orbite (r={ORBIT_RADIUS*100:.0f}cm)"),
    ]
    ax.legend(handles=handles, loc='lower right', fontsize=9,
              framealpha=0.92, edgecolor='gray')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(
        'NavMesh + Pathfinding A* — Mission lunaire SS3\n'
        f"Départ ({position_depart[0]:.2f}, {position_depart[1]:.2f}) "
        f"→ Orbite objets (r={ORBIT_RADIUS*100:.0f}cm) → Retour",
        fontsize=14, fontweight='bold', pad=15)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25, linewidth=0.5)
    plt.tight_layout()
    plt.show()
    # NOTE : Aucun plt.savefig() — pas de création de fichier PNG


# ===========================================================================
# TEST STANDALONE
# ===========================================================================

# ===========================================================================
# 10. TEST STANDALONE
# ===========================================================================

if __name__ == "__main__":
    from simulation.terrain_generator import generer_terrain
    from perception.ransac             import ransac
    from perception.DBSCAN             import dbscan
    from perception.filtration         import filtrer
    from navigation.triangulation      import perform_triangulation
    from config import (NOM_FICHIER, RANSAC_THRESHOLD,
                        DBSCAN_EPS, DBSCAN_MIN_SAMPLES)

    print("=" * 55)
    print("  Pipeline SS3 — Pathfinding A* + Orbite complète")
    print("=" * 55)

    points_bruts                            = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles_pts, _  = ransac(points_bruts, RANSAC_THRESHOLD)
    clusters                                = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
    objets_interet, obstacles               = filtrer(clusters)
    points_navmesh                          = np.vstack([sol, terrain_naturel])
    _, points_utilises, navigable, _        = perform_triangulation(points_navmesh)

    print(f"\nObjets d'intérêt : {len(objets_interet)}")
    print(f"Obstacles        : {len(obstacles)}")
    for obj in objets_interet + obstacles:
        print(f"  {obj}")

    grille, origine_xy, res = construire_grille(
        points_utilises, navigable, obstacles + objets_interet
    )
    print(f"\nGrille : {grille.shape[0]}x{grille.shape[1]} cellules "
          f"({res}m/cellule) — {(grille==0).sum()} cellules navigables")
    print(f"Coin bas-gauche terrain : ({origine_xy[0]:.3f}m, {origine_xy[1]:.3f}m)")

    # Position de départ : cellule libre la plus proche du coin bas-gauche
    ix_dep = int(0.5 / res)
    iy_dep = int(0.5 / res)
    cell_dep = cellule_libre_proche(grille, ix_dep, iy_dep)
    if cell_dep:
        position_depart = grille_vers_monde(*cell_dep, origine_xy, res)
    else:
        position_depart = (float(origine_xy[0] + 0.5), float(origine_xy[1] + 0.5))
    print(f"Position de départ rover : ({position_depart[0]:.3f}m, {position_depart[1]:.3f}m)\n")

    print("── Planification en cours ──")
    chemins, waypoints_monde, ordre, stats = planifier_mission(
        grille, origine_xy, res, objets_interet, position_depart=position_depart
    )

    # ── Résumé mission (sans imprimer tous les waypoints) ──
    print(f"\n{'=' * 55}")
    print(f"  RÉSUMÉ MISSION")
    print(f"{'=' * 55}")
    print(f"  Objets visités    : {len(ordre)}")
    print(f"  Total waypoints   : {len(waypoints_monde)}")
    print(f"  Distance totale   : {stats['distance_totale_m']:.2f} m")
    print(f"  Vitesse rover     : {stats['vitesse_ms']} m/s")
    print(f"  Temps estimé      : {stats['temps_str']}")
    print(f"{'=' * 55}\n")

    # ── Résumé par segment (pas de liste exhaustive) ──
    types_count = {}
    for wp in waypoints_monde:
        types_count[wp["type"]] = types_count.get(wp["type"], 0) + 1
    for t, n in types_count.items():
        print(f"  {t:10s} : {n} waypoints")

    # Export fichier texte complet
    exporter_waypoints(waypoints_monde)

    # Liste prête pour executer_chemin()
    coords_rover = waypoints_pour_rover(waypoints_monde)
    print(f"\n  {len(coords_rover)} coordonnées exportées pour executer_chemin()")
    print(f"  Premiers waypoints :")
    for i, (x, y) in enumerate(coords_rover[:5]):
        print(f"    WP{i:03d} : ({x:.4f}m, {y:.4f}m)")
    if len(coords_rover) > 5:
        print(f"    ... ({len(coords_rover)-5} autres dans mission_waypoints.txt)")

    # Visualisation (affichage écran, pas de PNG)
    plot_astar(grille, origine_xy, res,
               chemins, objets_interet, obstacles,
               waypoints_monde, ordre, position_depart=position_depart)