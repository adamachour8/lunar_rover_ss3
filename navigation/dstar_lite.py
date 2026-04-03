"""
navigation/dstar_lite.py
------------------------
Pathfinding D* Lite sur le NavMesh triangulé — SS3 Mission lunaire.

Référence : Koenig & Likhachev, « D* Lite », AAAI 2002.

Différence clé vs A* :
  - D* Lite planifie depuis le GOAL vers le START (sens inverse).
  - Il peut re-planifier efficacement si de nouveaux obstacles
    apparaissent en cours de mission (LiDAR dynamique).
  - Même interface de sortie qu'astar.py pour faciliter la comparaison.

Pipeline :
  1. Conversion grille numpy A* → GridWorld D* Lite
  2. Initialisation D* Lite (calculateKey, queue, rhs du goal = 0)
  3. computeShortestPath : propage les coûts jusqu'au start
  4. Extraction du chemin (greedy sur g)
  5. moveAndRescan : avancement + re-planification sur obstacle détecté
  6. Ordre de visite glouton + points de scan (= même logique qu'astar.py)
  7. planifier_mission_dstar : même signature que planifier_mission dans astar.py
  8. Visualisation 2D (même style PPT qu'astar.py)

Compatibilité :
  - Grille d'entrée : sortie de navigation/astar.construire_grille()
    -> np.array (nx, ny), convention grille[ix, iy]
    -> 0=navigable, 1=obstacle, 2=hors-carte
  - Objets : liste d'ObjetDetecte (perception/filtration.py)
  - Constantes : toutes depuis config.py
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
# 1. STRUCTURES : Node + GridWorld (améliorés par rapport à graph.py/grid.py)
# ===========================================================================

class Node:
    """
    Noeud D* Lite.
    Identique a graph.py mais avec 8-connectivite et cost() integre.
    """
    __slots__ = ('name', 'g', 'rhs', 'neighbors')

    def __init__(self, name: str):
        self.name      = name
        self.g         = float('inf')
        self.rhs       = float('inf')
        self.neighbors = []          # liste de noms (str)


class GridWorld:
    """
    Grille 2D pour D* Lite, construite depuis la grille numpy du NavMesh.

    Convention interne : cells[iy][ix]
      0  = libre
      1  = obstacle (ou hors-carte)

    Differences vs grid.py original :
      - 8-connectivite (diagonales) au lieu de 4
      - methode cost(a, b) : cout euclidien ou inf si obstacle
      - conversion depuis np.array (ix, iy) via from_numpy()
    """

    def __init__(self, nx: int, ny: int, cells: list):
        self.width  = nx
        self.height = ny
        self.cells  = cells          # [iy][ix]
        self.graph  = {}
        self.start  = None
        self.goal   = None
        self._build_graph()

    # ── Conversion nom <-> coordonnees ─────────────────────────────────

    @staticmethod
    def state_name(ix: int, iy: int) -> str:
        return f"x{ix}y{iy}"

    @staticmethod
    def state_coords(name: str):
        ix = int(name.split('x')[1].split('y')[0])
        iy = int(name.split('y')[1])
        return ix, iy

    # ── Construction graphe avec 8-connectivite ────────────────────────

    def _build_graph(self):
        for iy in range(self.height):
            for ix in range(self.width):
                self.graph[self.state_name(ix, iy)] = Node(self.state_name(ix, iy))

        for iy in range(self.height):
            for ix in range(self.width):
                node = self.graph[self.state_name(ix, iy)]
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx2, ny2 = ix + dx, iy + dy
                        if 0 <= nx2 < self.width and 0 <= ny2 < self.height:
                            node.neighbors.append(self.state_name(nx2, ny2))

    # ── Cout de deplacement ────────────────────────────────────────────

    def cost(self, name_a: str, name_b: str) -> float:
        """
        Cout euclidien entre deux noeuds adjacents.
        Retourne inf si name_b est obstacle ou hors-carte.
        """
        bx, by = self.state_coords(name_b)
        if self.cells[by][bx] != 0:
            return float('inf')
        ax, ay = self.state_coords(name_a)
        return np.sqrt((bx - ax) ** 2 + (by - ay) ** 2)   # 1.0 ou sqrt(2)

    def is_free(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height \
               and self.cells[iy][ix] == 0

    def setStart(self, s: str):
        self.start = s

    def setGoal(self, g: str):
        self.goal = g

    # ── Constructeur depuis grille numpy (sortie astar.construire_grille) ─

    @classmethod
    def from_numpy(cls, grille: np.ndarray) -> 'GridWorld':
        """
        Convertit np.array (nx, ny) [convention A*]
        en GridWorld [convention D* Lite cells[iy][ix]].

        grille[ix, iy] : 0=libre, 1=obstacle, 2=hors-carte
        cells[iy][ix]  : 0=libre, 1=obstacle
        """
        nx, ny = grille.shape
        cells = []
        for iy in range(ny):
            row = []
            for ix in range(nx):
                row.append(0 if int(grille[ix, iy]) == 0 else 1)
            cells.append(row)
        return cls(nx, ny, cells)


# ===========================================================================
# 2. ALGORITHME D* LITE (Koenig & Likhachev 2002)
# ===========================================================================

def _heuristic(a: str, b: str) -> float:
    """Distance euclidienne entre deux etats (heuristique admissible)."""
    ax, ay = GridWorld.state_coords(a)
    bx, by = GridWorld.state_coords(b)
    return np.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


def _calculate_key(graph: GridWorld, name: str, s_start: str, k_m: float):
    """
    Cle de priorite D* Lite.
    key = (min(g,rhs) + h(s_start, u) + k_m,  min(g,rhs))
    """
    n = graph.graph[name]
    m = min(n.g, n.rhs)
    return (m + _heuristic(s_start, name) + k_m, m)


def _update_vertex(graph: GridWorld, queue: list, in_queue: set,
                   name: str, s_start: str, k_m: float):
    """
    Met a jour le rhs d'un noeud et le re-insere dans la queue si necessaire.
    Utilise la lazy-deletion : on ne supprime pas physiquement les vieilles entrees.
    """
    n = graph.graph[name]

    # Le rhs du goal est toujours 0 (condition limite D* Lite)
    if name != graph.goal:
        n.rhs = float('inf')
        for nb_name in n.neighbors:
            c = graph.cost(name, nb_name)
            if c < float('inf'):
                candidate = graph.graph[nb_name].g + c
                if candidate < n.rhs:
                    n.rhs = candidate

    # Re-pousser dans la queue si localement inconsistant
    in_queue.discard(name)
    if n.g != n.rhs:
        key = _calculate_key(graph, name, s_start, k_m)
        heapq.heappush(queue, (key, name))
        in_queue.add(name)


def initDStarLite(graph: GridWorld, s_start: str, k_m: float = 0.0):
    """
    Initialise D* Lite :
      - Remet tous les g et rhs a inf
      - Pose rhs(goal) = 0
      - Insere le goal dans la queue prioritaire

    Retourne : queue (list), in_queue (set), k_m (float)

    Corrections vs version originale (d_star_lite.py uploade) :
      - La queue est correctement initialisee avec le goal et sa cle
      - k_m est retourne (et non ignore)
      - La signature ne prend plus 's_goal' en parametre separe
        car il est deja dans graph.goal
    """
    for n in graph.graph.values():
        n.g   = float('inf')
        n.rhs = float('inf')

    graph.graph[graph.goal].rhs = 0.0

    queue    = []
    in_queue = set()
    key = _calculate_key(graph, graph.goal, s_start, k_m)
    heapq.heappush(queue, (key, graph.goal))
    in_queue.add(graph.goal)

    return queue, in_queue, k_m


def computeShortestPath(graph: GridWorld, queue: list, in_queue: set,
                        s_start: str, k_m: float):
    """
    Boucle principale D* Lite.
    Propage les couts depuis le goal jusqu'a ce que s_start soit
    localement consistant (g == rhs).

    Cette fonction manquait completement dans la version originale :
    sans elle, initDStarLite n'avait aucun effet utile et moveAndRescan
    lisait des g = inf partout.
    """
    MAX_ITER = graph.width * graph.height * 4   # garde-fou anti-boucle infinie

    for _ in range(MAX_ITER):
        if not queue:
            break

        # Verifier si s_start est deja consistant
        start_key  = _calculate_key(graph, s_start, s_start, k_m)
        start_node = graph.graph[s_start]
        if queue[0][0] >= start_key and start_node.rhs == start_node.g:
            break

        key_old, u = heapq.heappop(queue)

        # Lazy deletion : ignorer les entrees perimees
        if u not in in_queue:
            continue
        in_queue.discard(u)

        key_new = _calculate_key(graph, u, s_start, k_m)
        node_u  = graph.graph[u]

        if key_old < key_new:
            # Cle perimee -> re-inserer avec la bonne cle
            heapq.heappush(queue, (key_new, u))
            in_queue.add(u)

        elif node_u.g > node_u.rhs:
            # Noeud sur-consistant -> le rendre consistant
            node_u.g = node_u.rhs
            for nb in node_u.neighbors:
                _update_vertex(graph, queue, in_queue, nb, s_start, k_m)

        else:
            # Noeud sous-consistant -> g = inf et re-propagation
            node_u.g = float('inf')
            _update_vertex(graph, queue, in_queue, u, s_start, k_m)
            for nb in node_u.neighbors:
                _update_vertex(graph, queue, in_queue, nb, s_start, k_m)


def moveAndRescan(graph: GridWorld, queue: list, in_queue: set,
                  s_current: str, k_m: float,
                  nouveaux_obstacles: list = None):
    """
    Avance d'un pas vers le goal.
    Si de nouveaux obstacles sont detectes, met a jour le graphe
    et re-planifie — c'est l'avantage cle de D* Lite sur A*.

    Parametre
    ---------
    nouveaux_obstacles : liste de noms de noeuds devenus obstacles
                         (None = pas de changement detecte ce pas)

    Retourne
    --------
    next_state : str  — prochain etat, ou 'goal' si arrive, ou None si bloque
    k_m        : float — accumulateur mis a jour

    Corrections vs version originale :
      - Appel a computeShortestPath avant de se deplacer
      - Mise a jour dynamique des obstacles (re-planification)
      - Cout euclidien (sqrt(2) pour diagonales) au lieu de g brut
      - k_m correctement mis a jour avec h(last_start, s_current)
    """
    if s_current == graph.goal:
        return 'goal', k_m

    # ── Re-planification sur obstacle dynamique ────────────────────────
    if nouveaux_obstacles:
        k_m += _heuristic(graph.start, s_current)
        graph.setStart(s_current)

        for obs_name in nouveaux_obstacles:
            ox, oy = GridWorld.state_coords(obs_name)
            graph.cells[oy][ox] = 1   # marquer obstacle
            node_obs = graph.graph[obs_name]
            for nb in node_obs.neighbors:
                _update_vertex(graph, queue, in_queue, nb, s_current, k_m)
            _update_vertex(graph, queue, in_queue, obs_name, s_current, k_m)

        computeShortestPath(graph, queue, in_queue, s_current, k_m)

    # ── Choisir le meilleur voisin (min g + cost) ──────────────────────
    node = graph.graph[s_current]
    best     = None
    best_val = float('inf')

    for nb_name in node.neighbors:
        c = graph.cost(s_current, nb_name)
        if c == float('inf'):
            continue
        val = graph.graph[nb_name].g + c
        if val < best_val:
            best_val = val
            best     = nb_name

    if best is None or best_val == float('inf'):
        return None, k_m   # chemin bloque

    return best, k_m


# ===========================================================================
# 3. EXTRACTION DU CHEMIN COMPLET
# ===========================================================================

def extraire_chemin(graph: GridWorld, s_start: str) -> list:
    """
    Reconstruit le chemin complet start -> goal
    en suivant le gradient de g (min g + cost a chaque pas).
    A appeler apres computeShortestPath.
    Retourne une liste de noms d'etats, ou None si aucun chemin.
    """
    chemin  = [s_start]
    current = s_start
    visited = set()
    limit   = graph.width * graph.height

    while current != graph.goal and limit > 0:
        limit  -= 1
        visited.add(current)
        node    = graph.graph[current]

        best     = None
        best_val = float('inf')
        for nb_name in node.neighbors:
            if nb_name in visited:
                continue
            c = graph.cost(current, nb_name)
            if c == float('inf'):
                continue
            val = graph.graph[nb_name].g + c
            if val < best_val:
                best_val = val
                best     = nb_name

        if best is None:
            return None   # chemin bloque

        chemin.append(best)
        current = best

    return chemin if current == graph.goal else None


# ===========================================================================
# 4. UTILITAIRES COORDONNEES (meme interface qu'astar.py)
# ===========================================================================

def monde_vers_grille(x, y, origine_xy, resolution=ASTAR_RESOLUTION):
    return int((x - origine_xy[0]) / resolution), int((y - origine_xy[1]) / resolution)

def grille_vers_monde(ix, iy, origine_xy, resolution=ASTAR_RESOLUTION):
    return origine_xy[0] + (ix + 0.5) * resolution, origine_xy[1] + (iy + 0.5) * resolution

def cellule_libre_proche(grille: np.ndarray, ix: int, iy: int):
    """Trouve la cellule libre numpy la plus proche de (ix, iy)."""
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
# 5. POINT DE SCAN & ORDRE GLOUTON (meme logique qu'astar.py)
# ===========================================================================

def point_scan(pos_rover_xy, centroide_objet_xy,
               scan_distance=ASTAR_SCAN_DISTANCE):
    direction = np.array(centroide_objet_xy) - np.array(pos_rover_xy)
    dist = np.linalg.norm(direction)
    if dist < scan_distance:
        return pos_rover_xy
    return np.array(pos_rover_xy) + (direction / dist) * (dist - scan_distance)


def ordre_visite_glouton(position_depart, objets):
    restants = list(objets)
    ordre    = []
    pos      = np.array(position_depart)
    while restants:
        idx = np.argmin([np.linalg.norm(pos - o.centroide[:2]) for o in restants])
        ordre.append(restants.pop(idx))
        pos = ordre[-1].centroide[:2]
    return ordre


# ===========================================================================
# 6. PLANIFICATION COMPLETE
#    Meme signature que planifier_mission() dans astar.py
# ===========================================================================

def planifier_mission_dstar(grille: np.ndarray, origine_xy: tuple,
                             resolution: float,
                             objets_interet: list,
                             position_depart=(0.0, 0.0)):
    """
    Depart -> scan objet 1 -> scan objet 2 -> ... -> retour origine.
    Utilise D* Lite au lieu de A*.

    Parametres
    ----------
    grille          : np.array (nx, ny) — sortie de astar.construire_grille()
    origine_xy      : (x_min, y_min) en metres
    resolution      : taille cellule en metres (= ASTAR_RESOLUTION depuis config)
    objets_interet  : liste d'ObjetDetecte (perception/filtration.py)
    position_depart : (x, y) en metres, defaut (0, 0)

    Retourne
    --------
    chemins   : liste de chemins — chaque chemin = liste de noms d'etats GridWorld
    waypoints : liste de (x, y) monde
    ordre     : liste d'ObjetDetecte dans l'ordre de visite
    """
    ordre     = ordre_visite_glouton(position_depart, objets_interet)
    chemins   = []
    waypoints = [position_depart]
    pos       = position_depart

    for obj in ordre:
        pt = tuple(point_scan(pos, obj.centroide[:2]))
        waypoints.append(pt)

        d_np = cellule_libre_proche(grille, *monde_vers_grille(*pos, origine_xy, resolution))
        f_np = cellule_libre_proche(grille, *monde_vers_grille(*pt,  origine_xy, resolution))

        if d_np is None or f_np is None:
            print(f"[D*] Objet {obj.label} — cellule introuvable")
            continue

        s_start = GridWorld.state_name(*d_np)
        s_goal  = GridWorld.state_name(*f_np)

        # Graphe frais pour chaque segment
        graph = GridWorld.from_numpy(grille)
        graph.setStart(s_start)
        graph.setGoal(s_goal)

        k_m = 0.0
        queue, in_queue, k_m = initDStarLite(graph, s_start, k_m)
        computeShortestPath(graph, queue, in_queue, s_start, k_m)

        chemin = extraire_chemin(graph, s_start)
        if chemin:
            chemins.append(chemin)
            pos = pt
            print(f"[D*] Objet {obj.label} ({obj.categorie}) — {len(chemin)} etapes")
        else:
            print(f"[D*] Objet {obj.label} — aucun chemin trouve")

    # Retour a l'origine
    d_np = cellule_libre_proche(grille, *monde_vers_grille(*pos,             origine_xy, resolution))
    f_np = cellule_libre_proche(grille, *monde_vers_grille(*position_depart, origine_xy, resolution))

    if d_np and f_np:
        s_start = GridWorld.state_name(*d_np)
        s_goal  = GridWorld.state_name(*f_np)

        graph = GridWorld.from_numpy(grille)
        graph.setStart(s_start)
        graph.setGoal(s_goal)

        k_m = 0.0
        queue, in_queue, k_m = initDStarLite(graph, s_start, k_m)
        computeShortestPath(graph, queue, in_queue, s_start, k_m)

        chemin = extraire_chemin(graph, s_start)
        if chemin:
            chemins.append(chemin)
            waypoints.append(position_depart)
            print(f"[D*] Retour origine — {len(chemin)} etapes")

    return chemins, waypoints, ordre


# ===========================================================================
# 7. CONVERSION CHEMIN D* (noms) -> COORDONNEES MONDE (pour visualisation)
# ===========================================================================

def chemin_vers_coords(chemin: list, origine_xy: tuple, resolution: float):
    """Convertit une liste de noms d'etats -> listes (xs, ys) en metres."""
    xs, ys = [], []
    for name in chemin:
        ix, iy = GridWorld.state_coords(name)
        x, y   = grille_vers_monde(ix, iy, origine_xy, resolution)
        xs.append(x)
        ys.append(y)
    return xs, ys


# ===========================================================================
# 8. VISUALISATION (meme style PPT qu'astar.py)
# ===========================================================================

COULEURS = ['#E63946', '#2196F3', '#FF9800', '#9C27B0', '#00BCD4', '#8BC34A']


def plot_dstar(grille: np.ndarray, origine_xy: tuple, resolution: float,
               chemins: list, objets_interet: list, obstacles: list,
               waypoints: list, ordre: list, position_depart=(0.0, 0.0)):
    """Vue 2D du dessus — NavMesh + chemins D* Lite — pour PPT."""
    fig, ax = plt.subplots(figsize=(13, 11))
    nx, ny  = grille.shape
    extent  = [origine_xy[0], origine_xy[0] + nx * resolution,
               origine_xy[1], origine_xy[1] + ny * resolution]

    cmap = ListedColormap(['#C8F5C8', '#FFCDD2', '#F0F0F0'])
    ax.imshow(grille.T, origin='lower', extent=extent,
              cmap=cmap, vmin=0, vmax=2, alpha=0.8, interpolation='nearest')

    for i, chemin in enumerate(chemins):
        c  = COULEURS[i % len(COULEURS)]
        lb = (f"Segment {i+1} -> Objet {ordre[i].label}"
              if i < len(ordre) else "Retour origine (0,0)")
        xs, ys = chemin_vers_coords(chemin, origine_xy, resolution)
        ax.plot(xs, ys, color=c, linewidth=2.5, zorder=4, label=lb)
        step = max(1, len(xs) // 8)
        for j in range(0, len(xs) - 1, step):
            ax.annotate('', xy=(xs[j+1], ys[j+1]), xytext=(xs[j], ys[j]),
                        arrowprops=dict(arrowstyle='->', color=c, lw=1.5))

    for obj in objets_interet:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=250, color='#1565C0', zorder=6, marker='*',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Objet {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8.5, color='#0D47A1',
                    bbox=dict(boxstyle='round,pad=0.3', fc='white',
                              ec='#1565C0', alpha=0.85))

    for obj in obstacles:
        ax.scatter(obj.centroide[0], obj.centroide[1],
                   s=200, color='#B71C1C', zorder=6, marker='X',
                   edgecolors='white', linewidth=0.5)
        ax.annotate(f" Obstacle {obj.label}\n {obj.hauteur*100:.0f}cm",
                    (obj.centroide[0], obj.centroide[1]),
                    fontsize=8, color='#B71C1C',
                    bbox=dict(boxstyle='round,pad=0.3', fc='white',
                              ec='#B71C1C', alpha=0.85))

    ax.scatter(*position_depart, s=350, color='#FF6F00',
               zorder=7, marker='D', edgecolors='white', linewidth=1)
    ax.annotate(' DEPART\n (0,0)', position_depart,
                fontsize=9, fontweight='bold', color='#E65100',
                bbox=dict(boxstyle='round,pad=0.3', fc='white',
                          ec='#FF6F00', alpha=0.9))

    handles = [
        mpatches.Patch(color='#C8F5C8', label='Zone navigable'),
        mpatches.Patch(color='#FFCDD2', label='Obstacle (zone gonflee >= rayon rover)'),
        mpatches.Patch(color='#F0F0F0', label='Hors-carte'),
        plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='#1565C0',
                   markersize=13, label="Objet d'interet"),
        plt.Line2D([0], [0], marker='X', color='w', markerfacecolor='#B71C1C',
                   markersize=11, label='Obstacle detecte'),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='#FF6F00',
                   markersize=11, label='Depart / Arrivee'),
    ]
    ax.legend(handles=handles, loc='lower right', fontsize=9,
              framealpha=0.92, edgecolor='gray')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(
        'NavMesh + Pathfinding D* Lite — Mission lunaire SS3\n'
        "Chemin optimal : Depart -> Objets d'interet -> Retour",
        fontsize=14, fontweight='bold', pad=15)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25, linewidth=0.5)
    plt.tight_layout()
    plt.savefig('navmesh_dstar.png', dpi=150, bbox_inches='tight')
    print("[plot] Sauvegarde : navmesh_dstar.png")
    plt.show()


# ===========================================================================
# TEST STANDALONE
# ===========================================================================

if __name__ == "__main__":
    from simulation.terrain_generator import generer_terrain
    from perception.ransac            import ransac
    from perception.DBSCAN            import dbscan
    from perception.filtration        import filtrer
    from navigation.triangulation     import perform_triangulation
    from navigation.astar             import construire_grille
    from config import (NOM_FICHIER, RANSAC_THRESHOLD,
                        DBSCAN_EPS, DBSCAN_MIN_SAMPLES)

    print("=== Pipeline SS3 — D* Lite ===\n")
    points_bruts                            = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles_pts, _  = ransac(points_bruts, RANSAC_THRESHOLD)
    clusters                                = dbscan(obstacles_pts, DBSCAN_EPS, DBSCAN_MIN_SAMPLES)
    objets_interet, obstacles               = filtrer(clusters)
    points_navmesh                          = np.vstack([sol, terrain_naturel])
    _, points_utilises, navigable, _        = perform_triangulation(points_navmesh)

    print(f"Objets d'interet : {len(objets_interet)}")
    print(f"Obstacles        : {len(obstacles)}")
    for obj in objets_interet + obstacles:
        print(f"  {obj}")

    # Reutilise construire_grille() d'astar.py — meme grille pour les deux algos
    grille, origine_xy, res = construire_grille(
        points_utilises, navigable, obstacles + objets_interet
    )
    print(f"\nGrille {grille.shape[0]}x{grille.shape[1]} "
          f"({res}m/cellule) — libres: {(grille==0).sum()}")

    chemins, waypoints, ordre = planifier_mission_dstar(
        grille, origine_xy, res, objets_interet, position_depart=(0.0, 0.0)
    )

    plot_dstar(grille, origine_xy, res,
               chemins, objets_interet, obstacles,
               waypoints, ordre, position_depart=(0.0, 0.0))