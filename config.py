# --- RANSAC ---
RANSAC_THRESHOLD     = 0.08   # Distance max (m) pour qu'un point soit considéré "sol"
RANSAC_BOSSEE_MAX    = 0.05   # Hauteur max (m) pour qu'une bosse soit terrain naturel (pas obstacle)

# --- DBSCAN ---
DBSCAN_EPS           = 0.25   # Distance max entre deux points du même cluster
DBSCAN_MIN_SAMPLES   = 8      # Nombre minimum de points pour former un cluster

# --- FILTRATION ---
FILTRE_HAUTEUR_MIN   = 0.05   # 5cm  — en dessous = bruit, ignoré
FILTRE_HAUTEUR_MAX   = 0.50   # 50cm — en dessous = objet d'intérêt, au dessus = obstacle
FILTRE_DISTANCE_MAX  = 8.0    # 8m   — ton nuage va jusqu'à ~7m en diagonale, 5m était trop petit

# --- TRIANGULATION ---
TRIANG_ANGLE_MAX     = 30     # Angle max de pente navigable en degrés
TRIANG_MAX_POINTS    = 5000   # Sous-échantillonnage si trop de points
TRIANG_LONGUEUR_MAX  = 0.3    # Longueur max (m) d'un côté de triangle — filtre les spikes
TRIANG_OUTLIER_VOISINS   = 10    # nombre de voisins à considérer
TRIANG_OUTLIER_DIST_MAX  = 0.2   # si le voisin le plus proche est à plus de 20cm, c'est un outlier

# --- FICHIERS ---
NOM_FICHIER          = "NuagePtsTest1-6.csv"