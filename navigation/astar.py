import numpy as np
import heapq
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (
    ASTAR_RESOLUTION, ASTAR_RAYON_ROVER, ASTAR_RAYON_INFLATION,
    ASTAR_SCAN_DISTANCE, ORBIT_RADIUS, ORBIT_N_POINTS, ORBIT_VITESSE_ROVER,
)


def construire_grille(points_navmesh, navigable, tous_les_objets,
                      resolution=ASTAR_RESOLUTION,
                      rayon_inflation=ASTAR_RAYON_INFLATION):
    assert rayon_inflation >= ASTAR_RAYON_ROVER, \
        f"rayon_inflation ({rayon_inflation}) doit être >= rayon du rover ({ASTAR_RAYON_ROVER})"

    x_min = points_navmesh[:, 0].min() - 0.5
    x_max = points_navmesh[:, 0].max() + 0.5
    y_min = points_navmesh[:, 1].min() - 0.5
    y_max = points_navmesh[:, 1].max() + 0.5

    nx = int(np.ceil((x_max - x_min) / resolution))
    ny = int(np.ceil((y_max - y_min) / resolution))

    grille = np.full((nx, ny), 2, dtype=np.int8)

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

    grille = _boucher_trous_horscarte(grille)
    return grille, (x_min, y_min), resolution


def _boucher_trous_horscarte(grille, taille_max_ilot=10):
    grille  = grille.copy()
    nx, ny  = grille.shape
    visite  = np.zeros((nx, ny), dtype=bool)

    for ix in range(nx):
        for iy in range(ny):
            if grille[ix, iy] != 2 or visite[ix, iy]:
                continue

            ilot        = []
            file        = [(ix, iy)]
            visite[ix, iy] = True
            touche_bord = False

            while file:
                cx, cy = file.pop()
                ilot.append((cx, cy))
                if cx == 0 or cx == nx - 1 or cy == 0 or cy == ny - 1:
                    touche_bord = True
                for ddx, ddy in [(-1,0),(1,0),(0,-1),(0,1),
                                  (-1,-1),(-1,1),(1,-1),(1,1)]:
                    nx2, ny2 = cx + ddx, cy + ddy
                    if 0 <= nx2 < nx and 0 <= ny2 < ny and not visite[nx2, ny2]:
                        if grille[nx2, ny2] == 2:
                            visite[nx2, ny2] = True
                            file.append((nx2, ny2))

            if not touche_bord and len(ilot) <= taille_max_ilot:
                for cx, cy in ilot:
                    grille[cx, cy] = 0

    return grille


def _point_dans_triangle(p, triangle):
    a, b, c    = triangle
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


def astar(grille, debut, fin):
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


def monde_vers_grille(x, y, origine_xy, resolution):
    return (int((x - origine_xy[0]) / resolution),
            int((y - origine_xy[1]) / resolution))

def grille_vers_monde(ix, iy, origine_xy, resolution):
    return (origine_xy[0] + (ix + 0.5) * resolution,
            origine_xy[1] + (iy + 0.5) * resolution)

def chemin_vers_monde(chemin, origine_xy, resolution):
    return [grille_vers_monde(ix, iy, origine_xy, resolution) for ix, iy in chemin]

def cellule_libre_proche(grille, ix, iy):
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


def generer_points_orbite(centroide_xy, rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS):
    cx, cy = centroide_xy
    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    return [(cx + rayon * np.cos(a), cy + rayon * np.sin(a)) for a in angles]


def orbite_complete_astar(grille, origine_xy, resolution,
                          centroide_xy, pos_rover_xy,
                          rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS):
    candidats = generer_points_orbite(centroide_xy, rayon, n_points)

    pts_valides = []
    for pt in candidats:
        ix, iy = monde_vers_grille(*pt, origine_xy, resolution)
        cell   = cellule_libre_proche(grille, ix, iy)
        if cell is not None:
            dist_snap = np.linalg.norm(
                np.array(grille_vers_monde(*cell, origine_xy, resolution)) - np.array(pt)
            )
            if dist_snap <= resolution * 4:
                pts_valides.append(pt)

    if len(pts_valides) < 2:
        print(f"  [Orbite] Pas assez de points navigables ({len(pts_valides)}) — orbite impossible")
        return [], [], 0

    dists      = [np.linalg.norm(np.array(pt) - np.array(pos_rover_xy)) for pt in pts_valides]
    idx_entree = int(np.argmin(dists))
    pts_ordonnes        = pts_valides[idx_entree:] + pts_valides[:idx_entree]
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
            continue
        if cell_a == cell_b:
            pts_confirmes.append(pt_b)
            n_ok += 1
            continue

        ch = astar(grille, cell_a, cell_b)
        if ch:
            chemins_orbite.append(ch)
            for wx, wy in chemin_vers_monde(ch, origine_xy, resolution)[1:]:
                pts_confirmes.append((wx, wy))
            n_ok += 1

    return chemins_orbite, pts_confirmes, n_ok


def ordre_visite_glouton(position_depart, objets):
    restants = list(objets)
    ordre    = []
    pos      = np.array(position_depart)
    while restants:
        idx = np.argmin([np.linalg.norm(pos - o.centroide[:2]) for o in restants])
        ordre.append(restants.pop(idx))
        pos = ordre[-1].centroide[:2]
    return ordre


def calculer_stats_mission(waypoints_monde, vitesse=ORBIT_VITESSE_ROVER):
    dist = 0.0
    for i in range(1, len(waypoints_monde)):
        dx = waypoints_monde[i]["x"] - waypoints_monde[i-1]["x"]
        dy = waypoints_monde[i]["y"] - waypoints_monde[i-1]["y"]
        dist += np.sqrt(dx**2 + dy**2)

    temps_s   = dist / vitesse if vitesse > 0 else 0.0
    minutes   = int(temps_s // 60)
    secondes  = temps_s % 60
    temps_str = f"{minutes}min {secondes:.1f}s" if minutes > 0 else f"{secondes:.1f}s"

    return {
        "distance_totale_m": round(dist, 3),
        "temps_s":           round(temps_s, 1),
        "temps_str":         temps_str,
        "vitesse_ms":        vitesse,
    }


def _estimer_duree_orbite(pts_orbite, vitesse=ORBIT_VITESSE_ROVER):
    dist = 0.0
    for i in range(1, len(pts_orbite)):
        dx = pts_orbite[i][0] - pts_orbite[i-1][0]
        dy = pts_orbite[i][1] - pts_orbite[i-1][1]
        dist += np.sqrt(dx**2 + dy**2)
    return dist / vitesse if vitesse > 0 else 60.0


def planifier_mission(grille, origine_xy, resolution,
                      objets_interet, position_depart=(0.0, 0.0),
                      envoyer_signal_ss2=True):
    if envoyer_signal_ss2:
        from communication.envoyer_roche import envoyer_roche, attendre_fin_photo
    else:
        def envoyer_roche(objet, position_xy, duree_orbite_s):
            print(f"  [SIM] Signal SS2 — roche {objet.label} "
                  f"({objet.hauteur*100:.1f}cm) orbite {duree_orbite_s:.1f}s")
            return True
        def attendre_fin_photo(label):
            print(f"  [SIM] Attente fin photo roche {label} — ignorée en simulation")
            return True

    ordre           = ordre_visite_glouton(position_depart, objets_interet)
    chemins         = []
    waypoints_monde = []

    waypoints_monde.append({
        "x": float(position_depart[0]), "y": float(position_depart[1]),
        "type": "depart", "label": "Départ",
    })

    pos = position_depart

    for obj in ordre:
        cx, cy = float(obj.centroide[0]), float(obj.centroide[1])

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
                    "type": "travel", "label": f"Approche Objet {obj.label}",
                })
            pos = pt_entree
            print(f"[A*] Approche Objet {obj.label} ({obj.categorie}) ✓")
        else:
            print(f"[A*] Objet {obj.label} — aucun chemin d'approche, objet ignoré")
            continue

        chemins_orbite, pts_orbite, n_ok = orbite_complete_astar(
            grille, origine_xy, resolution,
            (cx, cy), pos,
            rayon=ORBIT_RADIUS, n_points=ORBIT_N_POINTS
        )

        duree_orbite_s = _estimer_duree_orbite(pts_orbite) if chemins_orbite else (2 * np.pi * ORBIT_RADIUS) / ORBIT_VITESSE_ROVER
        envoyer_roche(obj, pos, duree_orbite_s)

        if chemins_orbite:
            chemins.extend(chemins_orbite)
            for wx, wy in pts_orbite:
                waypoints_monde.append({
                    "x": round(float(wx), 4), "y": round(float(wy), 4),
                    "type": "orbit", "label": f"Orbite Objet {obj.label} (r={ORBIT_RADIUS*100:.0f}cm)",
                })
            pos = pts_orbite[-1]
            print(f"[A*] Tour Objet {obj.label} ✓ — {n_ok}/{ORBIT_N_POINTS} segments, {len(pts_orbite)} waypoints")
        else:
            print(f"[A*] Objet {obj.label} — orbite impossible, scan simple (fallback)")
            direction = np.array([cx, cy]) - np.array(pos)
            dist      = np.linalg.norm(direction)
            if dist > ASTAR_SCAN_DISTANCE:
                pt_scan = np.array(pos) + (direction / dist) * (dist - ASTAR_SCAN_DISTANCE)
                waypoints_monde.append({
                    "x": round(float(pt_scan[0]), 4), "y": round(float(pt_scan[1]), 4),
                    "type": "scan", "label": f"Scan Objet {obj.label} (fallback)",
                })

        attendre_fin_photo(obj.label)

    d_cell = cellule_libre_proche(grille, *monde_vers_grille(*pos,             origine_xy, resolution))
    f_cell = cellule_libre_proche(grille, *monde_vers_grille(*position_depart, origine_xy, resolution))

    if d_cell and f_cell:
        ch_retour = astar(grille, d_cell, f_cell)
        if ch_retour:
            chemins.append(ch_retour)
            for wx, wy in chemin_vers_monde(ch_retour, origine_xy, resolution)[1:]:
                waypoints_monde.append({
                    "x": round(wx, 4), "y": round(wy, 4),
                    "type": "return", "label": "Retour départ",
                })
            print("[A*] Retour origine ✓")
        else:
            print("[A*] Retour origine — aucun chemin trouvé")
    else:
        print("[A*] Retour origine — cellule introuvable")

    waypoints_monde.append({
        "x": float(position_depart[0]), "y": float(position_depart[1]),
        "type": "arrivee", "label": "Arrivée",
    })

    stats = calculer_stats_mission(waypoints_monde, vitesse=ORBIT_VITESSE_ROVER)
    return chemins, waypoints_monde, ordre, stats


def exporter_waypoints(waypoints_monde, fichier="mission_waypoints.txt"):
    with open(fichier, "w", encoding="utf-8") as f:
        f.write("# Mission lunaire SS3 — Waypoints en coordonnées réelles (mètres)\n")
        f.write("# index\tx_m\ty_m\ttype\tlabel\n")
        for i, wp in enumerate(waypoints_monde):
            f.write(f"{i}\t{wp['x']:.4f}\t{wp['y']:.4f}\t{wp['type']}\t{wp['label']}\n")
    print(f"[Export] {len(waypoints_monde)} waypoints sauvegardés → {fichier}")


def waypoints_pour_rover(waypoints_monde):
    coords  = [(wp["x"], wp["y"]) for wp in waypoints_monde]
    seuil   = 0.01
    filtres = [coords[0]]
    for pt in coords[1:]:
        if np.linalg.norm(np.array(pt) - np.array(filtres[-1])) > seuil:
            filtres.append(pt)
    return filtres


def plot_astar(grille, origine_xy, resolution,
               chemins, objets_interet, obstacles,
               waypoints_monde, ordre, position_depart=(0.0, 0.0)):
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

    orbit_wps = [wp for wp in waypoints_monde if wp["type"] == "orbit"]
    if orbit_wps:
        ox = [wp["x"] for wp in orbit_wps]
        oy = [wp["y"] for wp in orbit_wps]
        ax.scatter(ox, oy, s=30, color='#FF6F00', zorder=5, marker='o',
                   label=f"Orbite (r={ORBIT_RADIUS*100:.0f}cm)", alpha=0.7)

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
                    bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#1565C0', alpha=0.85))

    for obj in obstacles:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=200, color='#B71C1C', zorder=6, marker='X',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Obstacle {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8, color='#B71C1C',
                    bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#B71C1C', alpha=0.85))

    ax.scatter(*position_depart, s=350, color='#FF6F00',
               zorder=7, marker='D', edgecolors='white', linewidth=1)
    ax.annotate(f' DÉPART\n ({position_depart[0]:.2f}, {position_depart[1]:.2f})',
                position_depart, fontsize=9, fontweight='bold', color='#E65100',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#FF6F00', alpha=0.9))

    handles = [
        mpatches.Patch(color='#C8F5C8', label='Zone navigable'),
        mpatches.Patch(color='#FFCDD2', label='Obstacle (zone gonflée)'),
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
    ax.legend(handles=handles, loc='lower right', fontsize=9, framealpha=0.92, edgecolor='gray')
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


if __name__ == "__main__":
    from simulation.terrain_generator import generer_terrain
    from perception.ransac             import ransac
    from perception.DBSCAN             import dbscan
    from perception.filtration         import filtrer
    from navigation.triangulation      import perform_triangulation
    from config import (NOM_FICHIER, RANSAC_THRESHOLD, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)

    points_bruts                           = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles_pts, _ = ransac(points_bruts, RANSAC_THRESHOLD)
    clusters                               = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
    objets_interet, obstacles              = filtrer(clusters)
    points_navmesh                         = np.vstack([sol, terrain_naturel])
    _, points_utilises, navigable, _       = perform_triangulation(points_navmesh)

    grille, origine_xy, res = construire_grille(
        points_utilises, navigable, obstacles + objets_interet
    )

    chemins, waypoints_monde, ordre, stats = planifier_mission(
        grille, origine_xy, res,
        objets_interet, position_depart=(0.0, 0.0),
        envoyer_signal_ss2=False
    )

    print(f"\nObjets visités  : {len(ordre)}")
    print(f"Total waypoints : {len(waypoints_monde)}")
    print(f"Distance totale : {stats['distance_totale_m']:.2f} m")
    print(f"Temps estimé    : {stats['temps_str']}")

    exporter_waypoints(waypoints_monde)
    plot_astar(grille, origine_xy, res,
               chemins, objets_interet, obstacles,
               waypoints_monde, ordre, position_depart=(0.0, 0.0))