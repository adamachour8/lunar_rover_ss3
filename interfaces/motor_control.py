import math
import serial
import time

def angle_entre_points(p1, p2):
    """Calcule l'angle en degrés entre deux points (x,y)."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx))

def distance_entre_points(p1, p2):
    """Distance euclidienne entre deux points."""
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def executer_chemin(waypoints, arduino, vitesse_ms_par_metre, vitesse_ms_par_degre):
    """
    Transforme une liste de waypoints (x,y) en commandes moteur.
    waypoints : [(x0,y0), (x1,y1), (x2,y2), ...]
    """
    angle_actuel = 0  # Direction actuelle du rover en degrés

    for i in range(1, len(waypoints)):
        p_actuel = waypoints[i-1]
        p_cible  = waypoints[i]

        # --- Calcul de la rotation nécessaire ---
        angle_cible = angle_entre_points(p_actuel, p_cible)
        delta_angle = angle_cible - angle_actuel

        # Normaliser entre -180 et +180
        delta_angle = (delta_angle + 180) % 360 - 180

        # --- Rotation ---
        if abs(delta_angle) > 2:  # Seuil de 2° pour ignorer micro-corrections
            duree_rotation = int(abs(delta_angle) * vitesse_ms_par_degre)
            if delta_angle > 0:
                envoyer_commande(arduino, f"GAUCHE:{duree_rotation}")
            else:
                envoyer_commande(arduino, f"DROITE:{duree_rotation}")
            angle_actuel = angle_cible

        # --- Avancer ---
        dist = distance_entre_points(p_actuel, p_cible)
        duree_avance = int(dist * vitesse_ms_par_metre)
        envoyer_commande(arduino, f"AVANT:{duree_avance}")

def envoyer_commande(arduino, commande):
    arduino.write(f"{commande}\n".encode())
    reponse = arduino.readline().decode().strip()
    print(f"Envoyé: {commande} → Reçu: {reponse}")
    return reponse