"""
Module de lecture de manette PS5 (DualSense) pour mode backup manuel.
Fournit les fonctions reutilisables pour init, lecture et conversion.
Aucune connexion Arduino ici -- c'est juste de la logique pure.
"""
import pygame
import time

# Indices typiques pour DualSense sur Linux/pygame -- verifies dans les tests
# (peuvent varier, on a un mode debug dans main_backup.py pour les remapper)
AXE_GAUCHE_Y  = 1   # joystick gauche, axe vertical (avancer/reculer)
AXE_DROIT_X   = 2   # joystick droit, axe horizontal (tourner)

BTN_CROIX     = 0   # X -- envoyer signal SS2
BTN_ROND      = 1   # O -- stop d'urgence
BTN_CARRE     = 2   # carre -- reset compteur objets manuels
BTN_TRIANGLE  = 3   # triangle -- quit propre
BTN_L1        = 9   # L1 -- diviser vitesse par 2
BTN_R1        = 10  # R1 -- doubler vitesse

DEAD_ZONE = 0.25    # ignore les petits mouvements parasites du joystick


def init_manette():
    """Initialise pygame et la premiere manette detectee. Retourne l'objet manette."""
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("Aucune manette detectee. Verifier connexion Bluetooth/USB.")

    manette = pygame.joystick.Joystick(0)
    manette.init()
    print(f"[Manette] Detectee : {manette.get_name()}")
    print(f"[Manette] {manette.get_numaxes()} axes, {manette.get_numbuttons()} boutons")
    return manette


def lire_etat(manette):
    """
    Lit l'etat actuel des joysticks et boutons.
    Retourne un dict avec axe_avant, axe_tourner, et l'etat des boutons.
    """
    pygame.event.pump()  # OBLIGATOIRE pour rafraichir l'etat des inputs

    # Inversion de Y parce que joystick vers le HAUT donne -1 par defaut
    axe_avant   = -manette.get_axis(AXE_GAUCHE_Y)
    axe_tourner = manette.get_axis(AXE_DROIT_X)

    # Application de la dead zone
    if abs(axe_avant) < DEAD_ZONE:
        axe_avant = 0.0
    if abs(axe_tourner) < DEAD_ZONE:
        axe_tourner = 0.0

    return {
        "axe_avant":   axe_avant,
        "axe_tourner": axe_tourner,
        "btn_croix":    manette.get_button(BTN_CROIX),
        "btn_rond":     manette.get_button(BTN_ROND),
        "btn_carre":    manette.get_button(BTN_CARRE),
        "btn_triangle": manette.get_button(BTN_TRIANGLE),
        "btn_l1":       manette.get_button(BTN_L1),
        "btn_r1":       manette.get_button(BTN_R1),
    }


def etat_vers_commande(etat, pas_distance, pas_angle):
    """
    Convertit l'etat des joysticks en commande Dist|Ang pour l'Arduino.
    Retourne None si aucun mouvement (joystick au centre).
    """
    if etat["axe_avant"] == 0.0 and etat["axe_tourner"] == 0.0:
        return None

    # Priorite : si on tourne, on tourne sur place. Sinon on avance.
    # Evite les mouvements en arc bizarres avec un firmware tic-par-tic.
    if abs(etat["axe_tourner"]) > abs(etat["axe_avant"]):
        ang  = pas_angle * etat["axe_tourner"]
        dist = 0.0
    else:
        dist = pas_distance * etat["axe_avant"]
        ang  = 0.0

    return f"Dist:{round(dist, 2)}|Ang:{round(ang, 1)}"


def fermer_manette():
    """Ferme proprement pygame."""
    pygame.quit()
    print("[Manette] Fermee")


# --- Mode debug pour identifier les indices d'axes/boutons ---
if __name__ == "__main__":
    print("Mode DEBUG -- bouge les joysticks et appuie sur les boutons.")
    print("Note les indices qui s'affichent et update les constantes en haut du fichier.")
    print("Ctrl+C pour quitter.\n")

    manette = init_manette()
    try:
        while True:
            pygame.event.pump()
            for i in range(manette.get_numaxes()):
                val = manette.get_axis(i)
                if abs(val) > 0.3:
                    print(f"  Axe {i} : {val:+.2f}")
            for i in range(manette.get_numbuttons()):
                if manette.get_button(i):
                    print(f"  Bouton {i} presse")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nSortie")
        fermer_manette()