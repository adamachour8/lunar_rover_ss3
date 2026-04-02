# --- RANSAC ---
RANSAC_THRESHOLD     = 0.08   # Distance max (m) pour qu'un point soit considéré "sol"
RANSAC_BOSSEE_MAX    = 0.05   # Hauteur max (m) pour qu'une bosse soit terrain naturel (pas obstacle)

# --- DBSCAN ---
DBSCAN_EPS           = 0.25   # Distance max entre deux points du même cluster
DBSCAN_MIN_SAMPLES   = 8      # Nombre minimum de points pour former un cluster

# --- FILTRATION ---
FILTRE_HAUTEUR_MIN   = 0.045   # 5cm  — en dessous = bruit, ignoré
FILTRE_HAUTEUR_MAX   = 0.505   # 50cm — en dessous = objet d'intérêt, au dessus = obstacle
FILTRE_DISTANCE_MAX  = 8.0    # 8m   — ton nuage va jusqu'à ~7m en diagonale, 5m était trop petit

# --- TRIANGULATION ---
TRIANG_ANGLE_MAX     = 30     # Angle max de pente navigable en degrés
TRIANG_MAX_POINTS    = 5000   # Sous-échantillonnage si trop de points
TRIANG_LONGUEUR_MAX  = 0.3    # Longueur max (m) d'un côté de triangle — filtre les spikes
TRIANG_OUTLIER_VOISINS   = 10    # nombre de voisins à considérer
TRIANG_OUTLIER_DIST_MAX  = 0.2   # si le voisin le plus proche est à plus de 20cm, c'est un outlier

# --- A* PATHFINDING ---
ASTAR_RESOLUTION     = 0.10   # Taille cellule grille (m) — descendre = plus précis mais plus lent
ASTAR_RAYON_ROVER    = 0.25   # Rayon du rover (m) — À AJUSTER selon specs réelles
ASTAR_RAYON_INFLATION= 0.40   # Doit toujours être >= ASTAR_RAYON_ROVER
ASTAR_SCAN_DISTANCE  = 0.70   # Distance à laquelle le rover s'arrête pour scanner (m)
ASTAR_Z_SOL          = 3.10   # Hauteur Z du sol (m) — pour filtrer les points hors-sol

# --- FICHIERS ---
NOM_FICHIER          = "NuagePtsTest1-6.csv"

# --- RELATION ARDUINO ---
VITESSE_MS_PAR_METRE = 5.00   # Temps (en ms) que prend le rover pour faire 1 m => estimation à 0.2 m/s
VITESSE_MS_PAR_DEGRE = 0.025  # Temps (en ms) que prend le rover pour tourner d'un degré => estimation à 40 degrés/s
NOM_PORT = 'COM3'          # Nom du port connecté à l'Arduino (lorsque connecté au RPi => '/dev/ttyUSB0' ou '/dev/ttyACM0' techniquement)