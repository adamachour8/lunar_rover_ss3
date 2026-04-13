import math
import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from interfaces.serial_utils import envoyer_commande, detecter_arduinos


def angle_entre_points(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx))


def distance_entre_points(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)


def executer_chemin(waypoints, arduino):
    angle_actuel = 0.0  # rover pointe vers l'Est (0 deg) au depart

    for i in range(1, len(waypoints)):
        p_actuel = waypoints[i-1]
        p_cible  = waypoints[i]

        angle_vise  = angle_entre_points(p_actuel, p_cible)
        delta_angle = angle_vise - angle_actuel
        delta_angle = (delta_angle + 180) % 360 - 180

        dist     = distance_entre_points(p_actuel, p_cible)
        commande = f"Dist:{round(dist, 2)}|Ang:{round(delta_angle, 1)}"

        print(f"\nEtape {i}: Vers {p_cible}")
        reponse = envoyer_commande(arduino, commande)

        if reponse == "D":
            angle_actuel = angle_vise
            print("Succes.")
        else:
            print("Erreur ou timeout de l'Arduino moteur !")
            return False
    return True


if __name__ == "__main__":
    arduino_moteur, _ = detecter_arduinos()
    if arduino_moteur is None:
        print("Arduino moteur introuvable")
        sys.exit(1)

    dossier_actuel = os.path.dirname(__file__)
    chemin_complet = os.path.join(dossier_actuel, '..', 'mission_waypoints.txt')
    data = np.loadtxt(chemin_complet, delimiter='\t', usecols=(1, 2), skiprows=2)
    chemin = [tuple(point) for point in data]

    executer_chemin(chemin, arduino_moteur)
    arduino_moteur.close()