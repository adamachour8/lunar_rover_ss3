# ===========================================================================
# config.py — Configuration Mission lunaire SS3
# ===========================================================================

# --- RANSAC ---
RANSAC_THRESHOLD     = 0.08   # Distance max (m) pour qu'un point soit considéré "sol"
RANSAC_BOSSEE_MAX    = 0.05   # Hauteur max (m) pour qu'une bosse soit terrain naturel

# --- DBSCAN ---
DBSCAN_EPS           = 0.25   # Distance max entre deux points du même cluster
DBSCAN_MIN_SAMPLES   = 8      # Nombre minimum de points pour former un cluster

# --- FILTRATION ---
FILTRE_HAUTEUR_MIN   = 0.05   # 5cm  — en dessous = bruit, ignoré
FILTRE_HAUTEUR_MAX   = 0.50   # 50cm — en dessous = objet d'intérêt, au dessus = obstacle
FILTRE_DISTANCE_MAX  = 8.0    # 8m   — ton nuage va jusqu'à ~7m en diagonale

# --- TRIANGULATION ---
TRIANG_ANGLE_MAX         = 30    # Angle max de pente navigable en degrés
TRIANG_MAX_POINTS        = 5000  # Sous-échantillonnage si trop de points
TRIANG_LONGUEUR_MAX      = 0.3   # Longueur max (m) d'un côté de triangle
TRIANG_OUTLIER_VOISINS   = 10    # Nombre de voisins à considérer pour filtre outliers
TRIANG_OUTLIER_DIST_MAX  = 0.2   # Distance max au voisin le plus proche (m)

# --- A* PATHFINDING ---
ASTAR_RESOLUTION     = 0.10   # Taille cellule grille (m) — descendre = plus précis mais plus lent
ASTAR_RAYON_ROVER    = 0.20   # Rayon du rover (m) — À AJUSTER selon specs réelles
ASTAR_RAYON_INFLATION= 0.35   # Doit toujours être >= ASTAR_RAYON_ROVER
ASTAR_SCAN_DISTANCE  = 0.50   # Distance à laquelle le rover s'arrête pour scanner (m)
ASTAR_Z_SOL          = 3.10   # Hauteur Z du sol (m) — pour filtrer les points hors-sol

# --- ORBITE AUTOUR DES OBJETS D'INTÉRÊT ---
# Le rover fait un tour COMPLET autour de chaque objet d'intérêt détecté.
# L'orbite est planifiée par A* entre chaque point consécutif du cercle,
# ce qui garantit un chemin navigable même si certains angles sont bloqués.

ORBIT_RADIUS         = 0.10   # Rayon d'orbite autour de l'objet (m) — 10 cm
                               # À ajuster selon la taille des objets attendus.
                               # Trop petit = risque de collision avec l'objet.
                               # Trop grand = risque de sortir de la zone navigable.

ORBIT_N_POINTS       = 16     # Nombre de points sur le cercle d'orbite.
                               # Plus élevé = tour plus lisse mais plus de calcul A*.
                               # Recommandé : 12–20 selon la résolution de la grille.
                               # À 0.10m/cellule et r=10cm, 16 points = ~3.9cm entre pts.

ORBIT_VITESSE_ROVER  = 0.10   # Vitesse de déplacement du rover (m/s).
                               # Utilisé pour estimer le temps de mission.
                               # À calibrer avec les tests réels sur le rover physique.

# --- GRILLE — POST-TRAITEMENT (bouchage des trous hors-carte) ---
ASTAR_TAILLE_MAX_ILOT = 10    # Taille max (cellules) d'un îlot hors-carte à boucher.
                               # Ces îlots sont des artefacts de scan LiDAR (trous isolés).
                               # À 0.10m/cell, 10 cellules ≈ zone de 10×10cm.
                               # Augmenter si des trous plus grands doivent être bouchés.

# --- FICHIERS ---
NOM_FICHIER          = "NuagePtsTest1-6.csv"
NOM_PORT             = "COM3"   # Port série Arduino — À AJUSTER selon votre système

