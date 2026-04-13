import socket
import json
import time
import sys, os
import serial
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SS2_IP, SS2_PORT, SS2_TIMEOUT, PHOTO_NB_PHOTOS, ORBIT_RADIUS


def envoyer_roche(objet, position_xy, duree_orbite_s):
    message = {
        "type":           "ROCHE_DETECTEE",
        "label":          int(objet.label),
        "hauteur_cm":     round(objet.hauteur * 100, 1),
        "rayon_orbite_m": round(ORBIT_RADIUS, 3),
        "nb_photos":      PHOTO_NB_PHOTOS,
        "duree_orbite_s": round(duree_orbite_s, 1),
        "position":  {"x": round(float(position_xy[0]), 4),
                      "y": round(float(position_xy[1]), 4)},
        "centroide": {"x": round(float(objet.centroide[0]), 4),
                      "y": round(float(objet.centroide[1]), 4)},
        "timestamp": round(time.time(), 3),
    }

    print(f"Envoi signal SS2 — roche {objet.label} ({objet.hauteur*100:.1f}cm)")

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(SS2_TIMEOUT)
            s.connect((SS2_IP, SS2_PORT))
            s.sendall(json.dumps(message).encode("utf-8"))
        print(f"  Message envoyé à SS2")
        return True
    except socket.timeout:
        print(f"Timeout SS2 ({SS2_TIMEOUT}s)")
        return False
    except ConnectionRefusedError:
        print(f"SS2 non disponible ({SS2_IP}:{SS2_PORT})")
        return False
    except Exception as e:
        print(f"Erreur comm SS2 : {e}")
        return False
    
def envoyer_roche_arduino(objet, position_xy, duree_orbite_s, arduino):
    message = {
        "hauteur_cm":     round(objet.hauteur * 100, 1),
        "rayon_orbite_m": round(ORBIT_RADIUS, 3),
        "nb_photos":      PHOTO_NB_PHOTOS,
        "duree_orbite_s": round(duree_orbite_s, 1),
        "position":  {"x": round(float(position_xy[0]), 4),
                      "y": round(float(position_xy[1]), 4)},
        "centroide": {"x": round(float(objet.centroide[0]), 4),
                      "y": round(float(objet.centroide[1]), 4)},
        "timestamp": round(time.time(), 3),
    }

    print(f"Envoi signal SS2 — roche {objet.label} ({objet.hauteur*100:.1f}cm)")

    commande = f"{message["hauteur_cm"]}|{message["position"["x"]]}|{message["position"["y"]]}|{message["centroide"["x"]]}|{message["centroide"["y"]]}"
    reponse = envoyer_commande_test(arduino, commande)

    if reponse == "D":
            print("Succès.")
    else:
        print("Erreur ou timeout de l'Arduino !")
        return False

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

# ================ Ajout récent ======================

if __name__ == "__main__":
    arduino = serial.Serial('COM4', baudrate=9600, timeout=5)
    time.sleep(2)

    # dossier_actuel = os.path.dirname(__file__)
    # chemin_complet = os.path.join(dossier_actuel, '..', 'mission_waypoints.txt')
    # data = np.loadtxt(chemin_complet, delimiter='\t', usecols=(1, 2), skiprows=2)
    
    # # data est maintenant une matrice (array) NumPy
    # # On peut le convertir en liste de tuples pour ton ancienne fonction :
    # chemin = [tuple(point) for point in data]

    # # Vider le buffer — lire le "PRET" initial avant d'envoyer PING
    # while arduino.in_waiting:
    #     ligne = arduino.readline().decode('utf-8').strip()
    #     print(f"[Init] {ligne}")

    # # Maintenant envoyer PING
    # arduino.write(b"PING\n")
    # reponse = arduino.readline().decode('utf-8').strip()
    # print(f"Connexion ELEGOO : {reponse}")

    # if reponse == "PONG_CAM":
    #     executer_chemin(chemin, arduino)
    # else:
    #     print("ELEGOO non disponible — vérifier la connexion USB")