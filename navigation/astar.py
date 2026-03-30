"""
navigation/astar.py
-------------------
Pathfinding A* sur le NavMesh triangulé.

Pipeline complet :
  1. Construction grille 2D depuis triangles navigables
  2. Inflation des obstacles (rayon de sécurité >= rayon du rover)
  3. A* pour trouver le chemin entre deux points
  4. Ordre de visite glouton (toujours l'objet le plus proche)
  5. Points de scan (s'arrêter à SCAN_DISTANCE de chaque objet)
  6. Retour à l'origine (0, 0)
"""

import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (
    ASTAR_RESOLUTION,
    ASTAR_RAYON_ROVER,
    ASTAR_RAYON_INFLATION,
    ASTAR_SCAN_DISTANCE,
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
        origine_xy : tuple (x_min, y_min)
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
        for gx in range(max(0, ix.min()), min(nx-1, ix.max()) + 1):
            for gy in range(max(0, iy.min()), min(ny-1, iy.max()) + 1):
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

    return grille, (x_min, y_min), resolution


def _point_dans_triangle(p, triangle):
    """Test si le point p est dans le triangle (3 sommets 2D)."""
    a, b, c   = triangle
    v0, v1, v2 = c-a, b-a, p-a
    d00, d01, d02 = np.dot(v0,v0), np.dot(v0,v1), np.dot(v0,v2)
    d11, d12      = np.dot(v1,v1), np.dot(v1,v2)
    denom = d00*d11 - d01*d01
    if abs(denom) < 1e-10:
        return False
    inv = 1.0 / denom
    u   = (d11*d02 - d01*d12) * inv
    v   = (d00*d12 - d01*d02) * inv
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
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

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
                gx, gy = x+dx, y+dy
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
    return int((x - origine_xy[0]) / resolution), int((y - origine_xy[1]) / resolution)

def grille_vers_monde(ix, iy, origine_xy, resolution):
    return origine_xy[0] + (ix+0.5)*resolution, origine_xy[1] + (iy+0.5)*resolution

def cellule_libre_proche(grille, ix, iy):
    """Trouve la cellule libre la plus proche de (ix, iy)."""
    nx, ny = grille.shape
    if 0 <= ix < nx and 0 <= iy < ny and grille[ix, iy] == 0:
        return ix, iy
    for rayon in range(1, 30):
        for dx in range(-rayon, rayon+1):
            for dy in range(-rayon, rayon+1):
                gx, gy = ix+dx, iy+dy
                if 0 <= gx < nx and 0 <= gy < ny and grille[gx, gy] == 0:
                    return gx, gy
    return None


# ===========================================================================
# 4. POINT DE SCAN
# ===========================================================================

def point_scan(pos_rover_xy, centroide_objet_xy,
               scan_distance=ASTAR_SCAN_DISTANCE):
    """
    Point d'arrêt pour scanner l'objet :
    sur la droite rover→objet, à scan_distance de l'objet.
    """
    direction = np.array(centroide_objet_xy) - np.array(pos_rover_xy)
    dist      = np.linalg.norm(direction)
    if dist < scan_distance:
        return pos_rover_xy
    return np.array(pos_rover_xy) + (direction/dist) * (dist - scan_distance)


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
# 6. PLANIFICATION COMPLÈTE
# ===========================================================================

def planifier_mission(grille, origine_xy, resolution,
                      objets_interet, position_depart=(0.0, 0.0)):
    """
    Départ → scan objet 1 → scan objet 2 → ... → retour origine.

    Returns:
        chemins  : liste de chemins (listes de (ix, iy))
        waypoints: liste de (x, y) monde
        ordre    : liste d'ObjetDetecte dans l'ordre de visite
    """
    ordre    = ordre_visite_glouton(position_depart, objets_interet)
    chemins  = []
    waypoints = [position_depart]
    pos       = position_depart

    for obj in ordre:
        pt = tuple(point_scan(pos, obj.centroide[:2]))
        waypoints.append(pt)
        d = cellule_libre_proche(grille, *monde_vers_grille(*pos, origine_xy, resolution))
        f = cellule_libre_proche(grille, *monde_vers_grille(*pt,  origine_xy, resolution))
        if d is None or f is None:
            print(f"[A*] Objet {obj.label} — cellule introuvable")
            continue
        ch = astar(grille, d, f)
        if ch:
            chemins.append(ch)
            pos = pt
            print(f"[A*] Objet {obj.label} ({obj.categorie}) — {len(ch)} étapes")
        else:
            print(f"[A*] Objet {obj.label} — aucun chemin trouvé")

    # Retour origine
    d = cellule_libre_proche(grille, *monde_vers_grille(*pos,            origine_xy, resolution))
    f = cellule_libre_proche(grille, *monde_vers_grille(*position_depart, origine_xy, resolution))
    if d and f:
        ch = astar(grille, d, f)
        if ch:
            chemins.append(ch)
            waypoints.append(position_depart)
            print(f"[A*] Retour origine — {len(ch)} étapes")

    return chemins, waypoints, ordre


# ===========================================================================
# 7. VISUALISATION PPT
# ===========================================================================

COULEURS = ['#E63946','#2196F3','#FF9800','#9C27B0','#00BCD4','#8BC34A']

def plot_astar(grille, origine_xy, resolution,
               chemins, objets_interet, obstacles,
               waypoints, ordre, position_depart=(0.0, 0.0)):
    """Vue 2D du dessus — NavMesh + chemins A* — pour PPT."""
    fig, ax = plt.subplots(figsize=(13, 11))
    nx, ny  = grille.shape
    extent  = [origine_xy[0], origine_xy[0]+nx*resolution,
               origine_xy[1], origine_xy[1]+ny*resolution]

    cmap = ListedColormap(['#C8F5C8', '#FFCDD2', '#F0F0F0'])
    ax.imshow(grille.T, origin='lower', extent=extent,
              cmap=cmap, vmin=0, vmax=2, alpha=0.8, interpolation='nearest')

    # Chemins avec flèches
    for i, chemin in enumerate(chemins):
        c  = COULEURS[i % len(COULEURS)]
        lb = (f"Segment {i+1} → Objet {ordre[i].label}"
              if i < len(ordre) else "Retour origine (0,0)")
        xs = [grille_vers_monde(p[0],p[1],origine_xy,resolution)[0] for p in chemin]
        ys = [grille_vers_monde(p[0],p[1],origine_xy,resolution)[1] for p in chemin]
        ax.plot(xs, ys, color=c, linewidth=2.5, zorder=4, label=lb)
        step = max(1, len(xs)//8)
        for j in range(0, len(xs)-1, step):
            ax.annotate('', xy=(xs[j+1],ys[j+1]), xytext=(xs[j],ys[j]),
                        arrowprops=dict(arrowstyle='->', color=c, lw=1.5))

    # Objets d'intérêt
    for obj in objets_interet:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=250, color='#1565C0', zorder=6, marker='*',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Objet {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8.5, color='#0D47A1',
                    bbox=dict(boxstyle='round,pad=0.3',fc='white',
                              ec='#1565C0', alpha=0.85))

    # Obstacles
    for obj in obstacles:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=200, color='#B71C1C', zorder=6, marker='X',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Obstacle {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8, color='#B71C1C',
                    bbox=dict(boxstyle='round,pad=0.3',fc='white',
                              ec='#B71C1C', alpha=0.85))

    # Départ
    ax.scatter(*position_depart, s=350, color='#FF6F00',
               zorder=7, marker='D', edgecolors='white', linewidth=1)
    ax.annotate(' DÉPART\n (0,0)', position_depart,
                fontsize=9, fontweight='bold', color='#E65100',
                bbox=dict(boxstyle='round,pad=0.3',fc='white',
                          ec='#FF6F00', alpha=0.9))

    # Légende
    handles = [
        mpatches.Patch(color='#C8F5C8', label='Zone navigable'),
        mpatches.Patch(color='#FFCDD2', label=f'Obstacle (zone gonflée ≥ rayon rover)'),
        mpatches.Patch(color='#F0F0F0', label='Hors-carte'),
        plt.Line2D([0],[0],marker='*',color='w',markerfacecolor='#1565C0',
                   markersize=13, label="Objet d'intérêt"),
        plt.Line2D([0],[0],marker='X',color='w',markerfacecolor='#B71C1C',
                   markersize=11, label='Obstacle détecté'),
        plt.Line2D([0],[0],marker='D',color='w',markerfacecolor='#FF6F00',
                   markersize=11, label='Départ / Arrivée'),
    ]
    ax.legend(handles=handles, loc='lower right', fontsize=9,
              framealpha=0.92, edgecolor='gray')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(
        'NavMesh + Pathfinding A* — Mission lunaire SS3\n'
        "Chemin optimal : Départ → Objets d'intérêt → Retour",
        fontsize=14, fontweight='bold', pad=15)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25, linewidth=0.5)
    plt.tight_layout()
    plt.savefig('navmesh_astar.png', dpi=150, bbox_inches='tight')
    print("[plot] Sauvegardé : navmesh_astar.png")
    plt.show()


# ===========================================================================
# TEST STANDALONE
# ===========================================================================

if __name__ == "__main__":
    from simulation.terrain_generator import generer_terrain
    from perception.ransac             import ransac
    from perception.DBSCAN             import dbscan
    from perception.filtration         import filtrer
    from navigation.triangulation      import perform_triangulation
    from config import (NOM_FICHIER, RANSAC_THRESHOLD,
                        DBSCAN_EPS, DBSCAN_MIN_SAMPLES)

    print("=== Pipeline SS3 — A* ===\n")
    points_bruts                            = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles_pts, _  = ransac(points_bruts, RANSAC_THRESHOLD)
    clusters                                = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
    objets_interet, obstacles               = filtrer(clusters)
    points_navmesh                          = np.vstack([sol, terrain_naturel])
    _, points_utilises, navigable, _        = perform_triangulation(points_navmesh)

    print(f"Objets d'intérêt : {len(objets_interet)}")
    print(f"Obstacles        : {len(obstacles)}")
    for obj in objets_interet + obstacles:
        print(f"  {obj}")

    grille, origine_xy, res = construire_grille(
        points_utilises, navigable, obstacles + objets_interet
    )
    print(f"\nGrille {grille.shape[0]}x{grille.shape[1]} "
          f"({res}m/cellule) — libres: {(grille==0).sum()}")

    chemins, waypoints, ordre = planifier_mission(
        grille, origine_xy, res, objets_interet, position_depart=(0.0, 0.0)
    )

    plot_astar(grille, origine_xy, res,
               chemins, objets_interet, obstacles,
               waypoints, ordre, position_depart=(0.0, 0.0))