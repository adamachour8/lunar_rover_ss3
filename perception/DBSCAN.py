import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import DBSCAN_EPS, DBSCAN_MIN_SAMPLES

from simulation.terrain_generator import generer_terrain
from perception.ransac import ransac
from config import NOM_FICHIER, RANSAC_THRESHOLD

def dbscan(obstacles, epsilon=DBSCAN_EPS, min_echantillons=DBSCAN_MIN_SAMPLES):
    """
    Regroupe les points obstacles en clusters.

    Args:
        obstacles        : np.array (N, 3) — points hors du sol (output RANSAC)
        epsilon          : Distance max entre deux points du même cluster
        min_echantillons : Nombre minimum de points pour former un cluster

    Returns:
        clusters : dict {label (int) -> np.array (N, 3)}
    """
    db     = DBSCAN(eps=epsilon, min_samples=min_echantillons).fit(obstacles)
    labels = db.labels_

    clusters = {
        label: obstacles[labels == label]
        for label in set(labels)
        if label != -1  # -1 = bruit, ignoré
    }

    return clusters


def plot_dbscan(clusters, points_all=None):
    """
    Affiche tous les clusters en couleurs différentes en 3D.

    Args:
        clusters   : dict {label -> np.array (N, 3)} — output de dbscan()
        points_all : np.array (N, 3) optionnel — tous les points originaux
                     (affichés en gris en arrière-plan si fourni)
    """
    fig = plt.figure()
    ax  = fig.add_subplot(projection='3d')

    if points_all is not None:
        ax.scatter(points_all[:,0], points_all[:,1], points_all[:,2],
                   color='lightgrey', s=1, alpha=0.3, label='Tous les points')

    for label, cluster_points in clusters.items():
        ax.scatter(cluster_points[:,0], cluster_points[:,1], cluster_points[:,2],
                   s=5, label=f'Cluster {label}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("DBSCAN — Clusters détectés")
    plt.legend(loc='upper left', markerscale=2, fontsize=7)
    plt.show()


if __name__ == "__main__":
    points_bruts                            = generer_terrain("simulation/" + NOM_FICHIER)
    sol, terrain_naturel, obstacles_pts, _  = ransac(points_bruts, RANSAC_THRESHOLD)
    clusters                                = dbscan(obstacles_pts)

    print(f"Nombre de clusters détectés : {len(clusters)}")
    for label, pts in clusters.items():
        print(f"  Cluster {label} : {len(pts)} points")

    plot_dbscan(clusters, points_all=obstacles_pts)