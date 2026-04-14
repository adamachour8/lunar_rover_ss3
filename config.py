import sys

# RANSAC
RANSAC_THRESHOLD     = 0.06 # avant 0.08
RANSAC_BOSSEE_MAX    = 0.03 # avant 0.05
RANSAC_SEED          = 42

# DBSCAN
DBSCAN_EPS           = 0.25
DBSCAN_MIN_SAMPLES   = 10 # avant 8

# Filtration
FILTRE_HAUTEUR_MIN   = 0.05
FILTRE_HAUTEUR_MAX   = 0.50
FILTRE_DISTANCE_MAX  = 8.0
FILTRE_COMPACITE_MAX = 5.0   # ratio max(longueur, largeur) / hauteur — filtre les cretes de terrain

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
ASTAR_SCAN_DISTANCE  = 0.60
ASTAR_Z_SOL          = 3.10

# Orbite
ORBIT_RADIUS         = 0.50
ORBIT_N_POINTS       = 16
ORBIT_VITESSE_ROVER  = 0.10
ORBIT_EXCLUSION_RADIUS = ORBIT_RADIUS - ASTAR_RESOLUTION  # zone bloquée autour des objets d'intérêt

# Grille
ASTAR_TAILLE_MAX_ILOT = 10

# Communication Arduino
ARDUINO_BAUDRATE = 9600
ARDUINO_TIMEOUT  = 20

# Auto-detection : on PINGue chaque port candidat et on identifie par la reponse
if sys.platform == "win32":
    PORTS_CANDIDATS = ["COM3", "COM4", "COM5", "COM6"]
else:
    # ttyACM* = Arduino officiel (Uno, Mega), ttyUSB* = clones CH340
    PORTS_CANDIDATS = [
        "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
    ]

# Reponses au PING (a coder cote firmware Arduino)
PONG_MOTEUR = "PONG_MOTEUR"
PONG_CAM    = "PONG_CAM"

# Conserves pour retrocompat si du vieux code y refere encore
NOM_PORT_MOTEUR = PORTS_CANDIDATS[0]
NOM_PORT_CAM    = PORTS_CANDIDATS[1] if len(PORTS_CANDIDATS) > 1 else PORTS_CANDIDATS[0]
# Fichiers
NOM_FICHIER = "Test_reel2.csv"

