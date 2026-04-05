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
            reponse = json.loads(s.recv(1024).decode("utf-8"))
            statut = reponse.get("status", "")

            if statut == "READY":
                print(f"SS2 prete — orbite peut commencer")
                return True
            elif statut == "ERROR":
                print(f"SS2 erreur : {reponse.get('message', '?')}")
                return False
            else:
                print(f"SS2 reponse inattendue : {reponse}")
                return False

    except socket.timeout:
        print(f"Timeout SS2 ({SS2_TIMEOUT}s) — orbite continue sans photo")
        return False
    except ConnectionRefusedError:
        print(f"SS2 non disponible ({SS2_IP}:{SS2_PORT})")
        return False
    except Exception as e:
        print(f"Erreur comm SS2 : {e}")
        return False


def attendre_fin_photo(label_roche):
    print(f"Attente fin photos roche {label_roche}...")

    demande = json.dumps({
        "type":  "DEMANDE_STATUT",
        "label": int(label_roche),
    }).encode("utf-8")

    for tentative in range(10):  # max 10 tentatives
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(SS2_TIMEOUT)
                s.connect((SS2_IP, SS2_PORT))
                s.sendall(demande)
                reponse = json.loads(s.recv(1024).decode("utf-8"))
                statut = reponse.get("status", "")

                if statut == "OK":
                    print(f"SS2 — {reponse.get('nb_photos_prises', '?')} photos prises")
                    return True
                elif statut == "EN_COURS":
                    print(f"SS2 en cours ({reponse.get('nb_photos_prises', '?')}/{reponse.get('nb_photos_total', '?')}) — attente...")
                    time.sleep(5)
                else:
                    print(f"SS2 statut inattendu : {reponse}")
                    return False

        except socket.timeout:
            print(f"Timeout tentative {tentative + 1}/10")
            return False
        except Exception as e:
            print(f"Erreur attente fin photo : {e}")
            return False

    print(f"SS2 — max tentatives atteint pour roche {label_roche}")
    return False