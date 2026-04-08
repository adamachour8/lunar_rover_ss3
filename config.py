# config.py — Mission lunaire SS3

# RANSAC
RANSAC_THRESHOLD     = 0.08
RANSAC_BOSSEE_MAX    = 0.05

# DBSCAN
DBSCAN_EPS           = 0.25
DBSCAN_MIN_SAMPLES   = 8

# Filtration
FILTRE_HAUTEUR_MIN   = 0.05
FILTRE_HAUTEUR_MAX   = 0.50
FILTRE_DISTANCE_MAX  = 8.0

# Triangulation
TRIANG_ANGLE_MAX         = 30
TRIANG_MAX_POINTS        = 5000
TRIANG_LONGUEUR_MAX      = 0.5
TRIANG_OUTLIER_VOISINS   = 10
TRIANG_OUTLIER_DIST_MAX  = 0.2

# A*
ASTAR_RESOLUTION     = 0.10
ASTAR_RAYON_ROVER    = 0.40
ASTAR_RAYON_INFLATION= 0.45  # doit être >= ASTAR_RAYON_ROVER
ASTAR_SCAN_DISTANCE  = 0.50
ASTAR_Z_SOL          = 3.10

# Orbite
ORBIT_RADIUS         = 0.75
ORBIT_N_POINTS       = 16
ORBIT_VITESSE_ROVER  = 0.10

# Grille
ASTAR_TAILLE_MAX_ILOT = 10

# Photogrammétrie
PHOTO_NB_PHOTOS      = 30

# Communication SS2
SS2_IP   = "10.0.0.2"   # IP du RPI SS2 sur le réseau Ethernet direct
SS2_PORT = 5005
SS2_TIMEOUT = 120  # 30 photos × ~2s = ~60s, marge à 120s

# Communication Arduino
NOM_PORT         = "/dev/ttyACM0"
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT  = 5

# Fichiers
NOM_FICHIER = "NuagePtsTest1-6.csv"

