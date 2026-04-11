import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import FILTRE_HAUTEUR_MIN, FILTRE_HAUTEUR_MAX, FILTRE_DISTANCE_MAX, FILTRE_COMPACITE_MAX


class ObjetDetecte:
    def __init__(self, label, points, categorie):
        self.label     = label
        self.points    = points
        self.categorie = categorie

        self.x_min, self.x_max = points[:,0].min(), points[:,0].max()
        self.y_min, self.y_max = points[:,1].min(), points[:,1].max()
        self.z_min, self.z_max = points[:,2].min(), points[:,2].max()

        self.longueur  = self.x_max - self.x_min
        self.largeur   = self.y_max - self.y_min
        self.hauteur   = self.z_max - self.z_min
        self.centroide = points.mean(axis=0)
        self.distance  = np.linalg.norm(self.centroide)

    def __repr__(self):
        return (
            f"[{self.categorie.upper()}] Cluster {self.label} | "
            f"Hauteur: {self.hauteur*100:.1f}cm | "
            f"Largeur: {self.largeur*100:.1f}cm | "
            f"Longueur: {self.longueur*100:.1f}cm | "
            f"Distance: {self.distance:.2f}m | "
            f"Centroïde: ({self.centroide[0]:.2f}, {self.centroide[1]:.2f}, {self.centroide[2]:.2f})"
        )


def filtrer(clusters):
    objets_interet = []
    obstacles      = []

    for label, points in clusters.items():
        hauteur   = points[:,2].max() - points[:,2].min()
        centroide = points.mean(axis=0)
        distance  = np.linalg.norm(centroide)

        if distance > FILTRE_DISTANCE_MAX:
            continue
        if hauteur < FILTRE_HAUTEUR_MIN:
            continue

        x_range = points[:,0].max() - points[:,0].min()
        y_range = points[:,1].max() - points[:,1].min()
        volume  = hauteur * x_range * y_range
        if volume < 0.0001:
            continue

        if hauteur <= FILTRE_HAUTEUR_MAX:
            compacite = max(x_range, y_range) / hauteur
            if compacite > FILTRE_COMPACITE_MAX:
                continue

        categorie = "interet" if hauteur <= FILTRE_HAUTEUR_MAX else "obstacle"
        objet     = ObjetDetecte(label, points, categorie)

        if categorie == "interet":
            objets_interet.append(objet)
        else:
            obstacles.append(objet)

    return objets_interet, obstacles