import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (TRIANG_ANGLE_MAX, TRIANG_MAX_POINTS, TRIANG_LONGUEUR_MAX,
                    TRIANG_OUTLIER_VOISINS, TRIANG_OUTLIER_DIST_MAX,
                    RANSAC_SEED, NOM_FICHIER, RANSAC_THRESHOLD)
from simulation.terrain_generator import generer_terrain
from perception.ransac import ransac


def filtrer_outliers(points_3d, n_voisins=TRIANG_OUTLIER_VOISINS, dist_max=TRIANG_OUTLIER_DIST_MAX):
    nbrs         = NearestNeighbors(n_neighbors=n_voisins + 1).fit(points_3d)
    dists, _     = nbrs.kneighbors(points_3d)
    dist_moyenne = dists[:, 1:].mean(axis=1)
    return points_3d[dist_moyenne < dist_max]


def perform_triangulation(points, angle_max=TRIANG_ANGLE_MAX):
    points_3d = np.array(points, dtype=float)

    if points_3d.ndim != 2 or points_3d.shape[1] != 3:
        raise ValueError("Points doit être de shape (N, 3).")
    if len(points_3d) < 3:
        raise ValueError("Minimum 3 points requis.")

    points_3d = filtrer_outliers(points_3d)
    print(f"[triangulation] Après filtre outliers : {len(points_3d)} points.")

    if len(points_3d) > TRIANG_MAX_POINTS:
        np.random.seed(RANSAC_SEED)
        idx       = np.random.choice(len(points_3d), TRIANG_MAX_POINTS, replace=False)
        points_3d = points_3d[idx]
        print(f"[triangulation] Sous-échantillonnage : {TRIANG_MAX_POINTS} points utilisés.")

    tri           = Delaunay(points_3d[:, :2])
    navigable     = []
    non_navigable = []

    for simplex in tri.simplices:
        p1, p2, p3 = points_3d[simplex]

        if (np.linalg.norm(p2 - p1) > TRIANG_LONGUEUR_MAX or
            np.linalg.norm(p3 - p2) > TRIANG_LONGUEUR_MAX or
            np.linalg.norm(p1 - p3) > TRIANG_LONGUEUR_MAX):
            continue

        normale = np.cross(p2 - p1, p3 - p1)
        norme   = np.linalg.norm(normale)
        if norme == 0:
            continue

        angle = np.degrees(np.arccos(np.clip(abs(normale[2] / norme), 0, 1)))

        if angle < angle_max:
            navigable.append(simplex)
        else:
            non_navigable.append(simplex)

    return tri, points_3d, np.array(navigable), np.array(non_navigable)


def plot_triangulation(points, navigable, non_navigable):
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection='3d')

    ax.scatter(points[:,0], points[:,1], points[:,2],
               color='steelblue', s=2, alpha=0.4, label='Points sol')

    nav_plotted = False
    for simplex in navigable:
        t = np.vstack([points[simplex], points[simplex][0]])
        ax.plot(t[:,0], t[:,1], t[:,2],
                color='green', alpha=0.4, linewidth=0.5,
                label='Navigable' if not nav_plotted else '')
        nav_plotted = True

    non_plotted = False
    for simplex in non_navigable:
        t = np.vstack([points[simplex], points[simplex][0]])
        ax.plot(t[:,0], t[:,1], t[:,2],
                color='red', alpha=0.5, linewidth=0.5,
                label='Non-navigable' if not non_plotted else '')
        non_plotted = True

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("Triangulation 3D — NavMesh lunaire")
    plt.legend(loc='upper left', fontsize=9)
    plt.show()


if __name__ == "__main__":
    points_bruts                             = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles, _       = ransac(points_bruts, RANSAC_THRESHOLD)
    points_navmesh                           = np.vstack([sol, terrain_naturel])
    tri, points_utilises, navigable, non_nav = perform_triangulation(points_navmesh)

    print(f"Triangles navigables     : {len(navigable)}")
    print(f"Triangles non-navigables : {len(non_nav)}")
    plot_triangulation(points_utilises, navigable, non_nav)