import socket
import json
import time
import sys, os
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