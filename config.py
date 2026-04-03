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
ASTAR_RESOLUTION     = 0.10   # Taille cellule grille (m)
ASTAR_RAYON_ROVER    = 0.20   # Rayon du rover (m)
ASTAR_RAYON_INFLATION= 0.35   # Doit toujours être >= ASTAR_RAYON_ROVER
ASTAR_SCAN_DISTANCE  = 0.50   # Distance à laquelle le rover s'arrête pour scanner (m)
ASTAR_Z_SOL          = 3.10   # Hauteur Z du sol (m)

# --- ORBITE AUTOUR DES OBJETS D'INTÉRÊT ---
ORBIT_RADIUS         = 0.10   # Rayon d'orbite autour de l'objet (m)
ORBIT_N_POINTS       = 16     # Nombre de points sur le cercle d'orbite
ORBIT_VITESSE_ROVER  = 0.10   # Vitesse de déplacement du rover (m/s)

# --- GRILLE — POST-TRAITEMENT ---
ASTAR_TAILLE_MAX_ILOT = 10    # Taille max (cellules) d'un îlot hors-carte à boucher

# --- PHOTOGRAMMÉTRIE (orbite caméra) ---
PHOTO_NB_PHOTOS      = 30     # Nombre de photos à prendre autour de la roche
                               # SS2 calcule automatiquement l'intervalle angulaire
                               # entre chaque photo selon ce nombre.

# --- COMMUNICATION SS2 (Raspberry Pi — Caméra / Photogrammétrie) ---
SS2_IP      = "192.168.1.20"  # IP Ethernet du Raspberry Pi SS2
                               # À AJUSTER selon le réseau de la mission.
SS2_PORT    = 5005             # Port TCP sur lequel SS2 écoute
SS2_TIMEOUT = 120              # Timeout max (s) pour attendre la fin de la session photo.
                               # 30 photos × ~2s chacune = ~60s typique.
                               # Mettre à 120s pour avoir une marge confortable.

# --- COMMUNICATION ARDUINO (Contrôle moteurs) ---
NOM_PORT             = "COM3"  # Port série Arduino — À AJUSTER selon votre système
                               # Linux/Mac : "/dev/ttyUSB0" ou "/dev/ttyACM0"
ARDUINO_BAUDRATE     = 9600    # Doit correspondre au Serial.begin() dans le sketch Arduino
ARDUINO_TIMEOUT      = 5       # Timeout (s) pour readline() sur le port série

# --- FICHIERS ---
NOM_FICHIER          = "NuagePtsTest1-6.csv"

