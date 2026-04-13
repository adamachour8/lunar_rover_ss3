"""
MODE BACKUP -- pilotage manuel du rover a la manette PS5.
A utiliser SI le mode autonome (main.py) echoue ou derive trop.
"""
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config import ORBIT_RADIUS
from interfaces.serial_utils     import envoyer_commande, detecter_arduinos
from interfaces.manette_control  import (
    init_manette, lire_etat, etat_vers_commande, fermer_manette
)
from communication.envoyer_roche import envoyer_roche_arduino

# === Parametres modifiables ===
PAS_DISTANCE_DEFAUT = 0.10   # 10 cm par tic
PAS_ANGLE_DEFAUT    = 15.0   # 15 deg par tic
TICK_HZ             = 5      # 5 commandes max par seconde
TIMEOUT_SECURITE_S  = 2.0    # arret si pas d'input pendant 2s
LABEL_MANUEL_BASE   = 9000   # labels manuels = 9001, 9002...


class FauxObjet:
    """Imite ObjetDetecte juste pour envoyer_roche_arduino()."""
    def __init__(self, label, hauteur_m=0.20):
        self.label     = label
        self.hauteur   = hauteur_m
        self.centroide = (0.60, 0.0, 0.0)  # roche supposee 60cm devant


def main():
    print("=" * 60)
    print("  MODE BACKUP -- Pilotage manuel manette PS5")
    print("=" * 60)
    print("  Joystick gauche Y    : avancer / reculer")
    print("  Joystick droit  X    : tourner")
    print("  X                    : envoyer signal SS2 (objet 1, 2, 3...)")
    print("  Carre                : reset compteur objets")
    print("  Rond                 : stop d'urgence")
    print("  Triangle             : quitter proprement")
    print("  L1 / R1              : vitesse / 2  ou  vitesse x 2")
    print("=" * 60)

    # --- Init manette ---
    try:
        manette = init_manette()
    except RuntimeError as e:
        print(f"ERREUR : {e}")
        sys.exit(1)

    # --- Init Arduinos (auto-detection) ---
    print("\nDetection des Arduinos...")
    arduino, arduino_cam = detecter_arduinos()
    if arduino is None:
        print("ERREUR : Arduino moteur introuvable.")
        fermer_manette()
        sys.exit(1)
    if arduino_cam is None:
        print("ATTENTION : Arduino SS2 cam introuvable. Bouton X desactive.")

    print("\nPret. Mode manuel actif.\n")

    # --- Etat de la session ---
    pas_distance        = PAS_DISTANCE_DEFAUT
    pas_angle           = PAS_ANGLE_DEFAUT
    compteur_objets     = 0
    derniere_activite   = time.time()
    btn_x_precedent     = False
    btn_carre_precedent = False
    btn_l1_precedent    = False
    btn_r1_precedent    = False

    try:
        while True:
            etat = lire_etat(manette)

            # --- Quit propre (triangle) ---
            if etat["btn_triangle"]:
                print("\n[Triangle] Quit demande")
                break

            # --- Stop d'urgence (rond) ---
            if etat["btn_rond"]:
                print("[Rond] STOP D'URGENCE -- moteurs arretes")
                envoyer_commande(arduino, "Dist:0|Ang:0")
                time.sleep(0.3)
                derniere_activite = time.time()
                continue

            # --- Reset compteur objets (carre, front montant) ---
            if etat["btn_carre"] and not btn_carre_precedent:
                compteur_objets = 0
                print("[Carre] Compteur objets remis a zero")
            btn_carre_precedent = etat["btn_carre"]

            # --- Modif vitesse (L1 / R1, front montant) ---
            if etat["btn_l1"] and not btn_l1_precedent:
                pas_distance /= 2
                pas_angle    /= 2
                print(f"[L1] Vitesse / 2 -> dist={pas_distance:.3f}m, ang={pas_angle:.1f}deg")
            btn_l1_precedent = etat["btn_l1"]

            if etat["btn_r1"] and not btn_r1_precedent:
                pas_distance *= 2
                pas_angle    *= 2
                print(f"[R1] Vitesse x 2 -> dist={pas_distance:.3f}m, ang={pas_angle:.1f}deg")
            btn_r1_precedent = etat["btn_r1"]

            # --- Envoi signal SS2 (X, front montant) ---
            if etat["btn_croix"] and not btn_x_precedent:
                compteur_objets += 1
                label = LABEL_MANUEL_BASE + compteur_objets
                print(f"\n[X] Signal SS2 manuel - Objet #{compteur_objets} (label={label})")
                if arduino_cam is not None:
                    faux = FauxObjet(label=label, hauteur_m=0.20)
                    envoyer_roche_arduino(faux, position_xy=(0.0, 0.0), arduino_cam=arduino_cam)
                else:
                    print("[X] Pas d'Arduino cam - signal ignore")
                print()
            btn_x_precedent = etat["btn_croix"]

            # --- Mouvement (joysticks) ---
            commande = etat_vers_commande(etat, pas_distance, pas_angle)
            if commande is not None:
                envoyer_commande(arduino, commande)
                derniere_activite = time.time()

            # --- Timeout securite ---
            if time.time() - derniere_activite > TIMEOUT_SECURITE_S:
                envoyer_commande(arduino, "Dist:0|Ang:0")
                derniere_activite = time.time()

            time.sleep(1.0 / TICK_HZ)

    except KeyboardInterrupt:
        print("\n[Ctrl+C] Sortie forcee")

    finally:
        print("\nFermeture...")
        try:
            envoyer_commande(arduino, "Dist:0|Ang:0")
        except Exception:
            pass
        arduino.close()
        if arduino_cam is not None:
            arduino_cam.close()
        fermer_manette()
        print("Mode backup termine.")


if __name__ == "__main__":
    main()