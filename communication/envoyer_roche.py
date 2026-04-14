"""
Envoi des donnees roche a l'Arduino SS2.
L'Arduino SS2 controle un moteur qui fait tourner et monter/baisser une tige
porteuse de camera. Les photos sont prises a la main par telephone (bluetooth)
pendant que le rover orbite.

MODE FIRE-AND-FORGET : on envoie les donnees a SS2 mais on n'attend JAMAIS
de reponse. La mission continue no matter what.
"""
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import ORBIT_RADIUS


def envoyer_roche_arduino(objet, position_xy, arduino_cam):
    """
    Format envoye : largeur_max|hauteur|rover_x|rover_y|cent_x|cent_y|rayon_m|
    Fire-and-forget : on ecrit sur le port serie et on continue immediatement,
    sans jamais attendre de reponse de SS2.
    """
    if arduino_cam is None:
        return True

    hauteur     = round(objet.hauteur, 4)
    largeur_max = round(max(objet.longueur, objet.largeur), 4)
    rover_x     = round(float(position_xy[0]), 4)
    rover_y     = round(float(position_xy[1]), 4)
    cent_x      = round(float(objet.centroide[0]), 4)
    cent_y      = round(float(objet.centroide[1]), 4)
    rayon_m     = round(ORBIT_RADIUS, 3)

    commande = f"{largeur_max}|{hauteur}|{rover_x}|{rover_y}|{cent_x}|{cent_y}|{rayon_m}|"
    print(f"[SS2] Roche {objet.label} ({hauteur}m) -> {commande} (fire-and-forget)")

    try:
        # Vider le buffer RX pour eviter qu'il sature avec les vieilles reponses
        if arduino_cam.in_waiting:
            arduino_cam.reset_input_buffer()
        arduino_cam.write(f"{commande}\n".encode('utf-8'))
        arduino_cam.flush()
    except Exception as e:
        print(f"[SS2] Erreur envoi (ignoree) : {e}")

    return True


def fin_orbite_arduino(arduino_cam):
    """
    Signale a l'Arduino SS2 que l'orbite est terminee.
    Fire-and-forget : aucune attente de reponse.
    """
    if arduino_cam is None:
        return True

    try:
        if arduino_cam.in_waiting:
            arduino_cam.reset_input_buffer()
        arduino_cam.write(b"FIN_ORBITE\n")
        arduino_cam.flush()
        print("[SS2] FIN_ORBITE envoye (fire-and-forget)")
    except Exception as e:
        print(f"[SS2] Erreur fin orbite (ignoree) : {e}")

    return True