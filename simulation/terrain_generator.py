import numpy as np

def generer_terrain(filepath):
    """
    Lit un nuage de points depuis un fichier CSV (X, Y, Z, R, G, B).
    Retourne un array numpy (N, 3) avec seulement X, Y, Z.
    """
    data   = np.loadtxt(filepath, delimiter=",")
    points = data[:, :3]
    return points
        