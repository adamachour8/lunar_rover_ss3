import math
import os
import serial
import time
import numpy as np

#===================== Ajouts récents ======================

def envoyer_commande_test(arduino, commande):
    arduino.write(f"{commande}\n".encode('utf-8'))
    
    # Lire jusqu'à recevoir "D" ou "PONG" — ignorer les lignes debug
    while True:
        reponse = arduino.readline().decode('utf-8').strip()
        if reponse in ["D", "PONG", "PRET"]:
            print(f"  Envoyé : {commande} → Reçu : {reponse}")
            return reponse
        elif reponse.startswith("#"):
            print(f"  [Arduino debug] {reponse}")
#==================================================================

def angle_entre_points(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    # Retourne l'angle absolu par rapport à l'axe X (Est)
    return math.degrees(math.atan2(dy, dx))

def distance_entre_points(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def executer_chemin(waypoints, arduino):
    # On assume que le rover pointe vers l'Est (0°) au départ
    angle_actuel = 0.0 

    for i in range(1, len(waypoints)):
        p_actuel = waypoints[i-1]
        p_cible  = waypoints[i]

        # 1. Calculer l'angle vers lequel on doit pointer
        angle_vise = angle_entre_points(p_actuel, p_cible)
        
        # 2. Calculer de combien on doit tourner (Relatif)
        delta_angle = angle_vise - angle_actuel
        
        # Normalisation entre -180 et +180
        delta_angle = (delta_angle + 180) % 360 - 180

        # 3. Calculer la distance à parcourir
        dist = distance_entre_points(p_actuel, p_cible)

        # 4. Envoyer UNE SEULE commande complète
        commande = f"Dist:{round(dist, 2)}|Ang:{round(delta_angle, 1)}"
        
        print(f"\nÉtape {i}: Vers {p_cible}")
        reponse = envoyer_commande_test(arduino, commande)

        if reponse == "D":
            # Mise à jour de l'orientation du rover APRÈS le mouvement réussi
            angle_actuel = angle_vise
            print("Succès.")
        else:
            print("Erreur ou timeout de l'Elegoo !")
            break

def envoyer_commande(arduino, commande):
    arduino.write(f"{commande}\n".encode('utf-8'))
    
    # On attend la réponse "D"
    reponse = arduino.readline().decode('utf-8').strip()
    print(f"Envoyé: {commande} -> Reçu: {reponse}")
    return reponse

# ================ Ajout récent ======================

if __name__ == "__main__":
    arduino = serial.Serial('COM3', baudrate=9600, timeout=5)
    time.sleep(2)

    dossier_actuel = os.path.dirname(__file__)
    chemin_complet = os.path.join(dossier_actuel, '..', 'mission_waypoints.txt')
    data = np.loadtxt(chemin_complet, delimiter='\t', usecols=(1, 2), skiprows=2)
    
    # data est maintenant une matrice (array) NumPy
    # On peut le convertir en liste de tuples pour ton ancienne fonction :
    chemin = [tuple(point) for point in data]

    # Vider le buffer — lire le "PRET" initial avant d'envoyer PING
    while arduino.in_waiting:
        ligne = arduino.readline().decode('utf-8').strip()
        print(f"[Init] {ligne}")

    # Maintenant envoyer PING
    arduino.write(b"PING\n")
    reponse = arduino.readline().decode('utf-8').strip()
    print(f"Connexion ELEGOO : {reponse}")

    if reponse == "PONG":
        executer_chemin(chemin, arduino)
    else:
        print("ELEGOO non disponible — vérifier la connexion USB")