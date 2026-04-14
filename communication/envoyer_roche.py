"""
Envoi des donnees roche a l'Arduino SS2.
L'Arduino SS2 controle un moteur qui fait tourner et monter/baisser une tige
porteuse de camera. Les photos sont prises a la main par telephone (bluetooth)
pendant que le rover orbite.
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import ORBIT_RADIUS
from interfaces.serial_utils import envoyer_commande


def envoyer_roche_arduino(objet, position_xy, arduino_cam):
    """
    Format envoye : largeur_max|hauteur|rover_x|rover_y|cent_x|cent_y|rayon_m
    L'Arduino calcule son angle de pointage et son elevation, puis repond 'D'.
    """
    hauteur     = round(objet.hauteur, 4)
    largeur_max = round(max(objet.longueur, objet.largeur), 4)
    rover_x     = round(float(position_xy[0]), 4)
    rover_y     = round(float(position_xy[1]), 4)
    cent_x      = round(float(objet.centroide[0]), 4)
    cent_y      = round(float(objet.centroide[1]), 4)
    rayon_m     = round(ORBIT_RADIUS, 3)

    commande = f"{largeur_max}|{hauteur}|{rover_x}|{rover_y}|{cent_x}|{cent_y}|{rayon_m}|"
    print(f"[SS2] Roche {objet.label} ({hauteur}m) -> {commande}")

    reponse = envoyer_commande(arduino_cam, commande)
    if reponse == "D":
        print(f"[SS2] Camera positionnee pour roche {objet.label}")
        return True
    print(f"[SS2] Echec positionnement camera (reponse={reponse})")
    return False

def fin_orbite_arduino(arduino_cam):
    """
    Signale à l'Arduino SS2 que l'orbite est terminée.
    """
    reponse = envoyer_commande(arduino_cam, "FIN_ORBITE")
    if reponse == "D":
        print("[SS2] Fin orbite confirmee")
        return True
    print(f"[SS2] Echec signal fin orbite (reponse={reponse})")
    return False