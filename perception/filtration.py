import numpy as np

# CONSTANTES ----------------------------------------------------------------

HAUTEUR_MIN = 0.05   # 5cm
HAUTEUR_MAX = 0.50   # 50cm
DISTANCE_MAX = 5.0   # 5m de l'origine

# CLASSES -------------------------------------------------------------------

class ObjetDetecte:
    def __init__(self, label, points, categorie):
        self.label      = label
        self.points     = points
        self.categorie  = categorie  # "interet" ou "obstacle"

        # Bounding box
        self.x_min, self.x_max = points[:, 0].min(), points[:, 0].max()
        self.y_min, self.y_max = points[:, 1].min(), points[:, 1].max()
        self.z_min, self.z_max = points[:, 2].min(), points[:, 2].max()

        self.longueur  = self.x_max - self.x_min  # en mètres
        self.largeur   = self.y_max - self.y_min
        self.hauteur   = self.z_max - self.z_min

        # Centroïde
        self.centroide = points.mean(axis=0)  # [x, y, z]

        # Distance à l'origine
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


# FONCTION PRINCIPALE -------------------------------------------------------

def filtrer(clusters):
    """
    Prend le dictionnaire de clusters DBSCAN et retourne deux listes :
    - objets_interet  : hauteur entre 5cm et 50cm, distance <= 5m
    - obstacles       : hauteur > 50cm, distance <= 5m
    Les objets trop petits (<5cm) ou trop loin (>5m) sont ignorés.
    """

    objets_interet = []
    obstacles      = []

    for label, points in clusters.items():

        # Calcul hauteur et centroïde
        hauteur  = points[:, 2].max() - points[:, 2].min()
        centroide = points.mean(axis=0)
        distance  = np.linalg.norm(centroide)

        # Filtres de rejet
        if distance > DISTANCE_MAX:
            continue  # trop loin
        if hauteur < HAUTEUR_MIN:
            continue  # bruit

        # Filtre volume
        x_range = points[:, 0].max() - points[:, 0].min()
        y_range = points[:, 1].max() - points[:, 1].min()
        volume = hauteur * x_range * y_range
        if volume < 0.0001:
            continue

        # Classification
        if hauteur <= HAUTEUR_MAX:
            categorie = "interet"
        else:
            categorie = "obstacle"

        objet = ObjetDetecte(label, points, categorie)

        if categorie == "interet":
            objets_interet.append(objet)
        else:
            obstacles.append(objet)

    return objets_interet, obstacles