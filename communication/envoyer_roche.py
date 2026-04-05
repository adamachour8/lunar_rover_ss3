import socket
import json
import time
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SS2_IP, SS2_PORT, SS2_TIMEOUT, PHOTO_NB_PHOTOS, ORBIT_RADIUS


def envoyer_roche(objet, position_xy, duree_orbite_s):
    """
    Envoie les informations de la roche à SS2 et attend sa confirmation.

    Args:
        objet          : ObjetDetecte — la roche détectée par SS3
        position_xy    : tuple (x, y) — position du rover au moment du signal (m)
        duree_orbite_s : float — durée estimée de l'orbite complète (s)
                         Calculée par planifier_mission() via calculer_stats_mission()

    Returns:
        True  si SS2 a confirmé "READY" (orbite peut commencer)
        False si timeout ou SS2 non disponible (mission continue en mode dégradé)
    """
    message = {
        "type":          "ROCHE_DETECTEE",
        "label":         int(objet.label),
        "hauteur_cm":    round(objet.hauteur * 100, 1),
        "rayon_orbite_m": round(ORBIT_RADIUS, 3),
        "nb_photos":     PHOTO_NB_PHOTOS,
        "duree_orbite_s": round(duree_orbite_s, 1),
        # SS2 peut calculer : intervalle_photo_s = duree_orbite_s / nb_photos
        "position": {
            "x": round(float(position_xy[0]), 4),
            "y": round(float(position_xy[1]), 4),
        },
        "centroide": {
            "x": round(float(objet.centroide[0]), 4),
            "y": round(float(objet.centroide[1]), 4),
        },
        "timestamp": round(time.time(), 3),
    }

    print(f"\n📡 Envoi signal à SS2 — Roche {objet.label} "
          f"({objet.hauteur*100:.1f}cm) — "
          f"orbite {duree_orbite_s:.1f}s — "
          f"{PHOTO_NB_PHOTOS} photos prévues")

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(SS2_TIMEOUT)
            s.connect((SS2_IP, SS2_PORT))
            s.sendall(json.dumps(message).encode("utf-8"))

            # Attente de la confirmation "READY" — SS2 a réglé la caméra
            reponse_raw = s.recv(1024)
            reponse = json.loads(reponse_raw.decode("utf-8"))

            statut = reponse.get("status", "")

            if statut == "READY":
                print(f"✅ SS2 prête — caméra réglée pour {objet.hauteur*100:.1f}cm "
                      f"— orbite peut commencer")
                return True

            elif statut == "ERROR":
                print(f"⚠️  SS2 a signalé une erreur : {reponse.get('message', '?')} "
                      f"— mission continue sans photo")
                return False

            else:
                print(f"⚠️  Réponse SS2 inattendue : {reponse} — mission continue")
                return False

    except socket.timeout:
        print(f"⚠️  Timeout SS2 ({SS2_TIMEOUT}s) — caméra non disponible, "
              f"orbite continue sans photo")
        return False

    except ConnectionRefusedError:
        print(f"⚠️  SS2 non disponible ({SS2_IP}:{SS2_PORT}) — "
              f"orbite continue en mode dégradé")
        return False

    except Exception as e:
        print(f"⚠️  Erreur comm SS2 : {e} — mission continue")
        return False


def attendre_fin_photo(label_roche):
    """
    Attend que SS2 confirme que les 30 photos sont prises.
    Appelée APRÈS que l'orbite est terminée.

    Args:
        label_roche : int — identifiant de la roche (pour le log)

    Returns:
        True  si SS2 a confirmé "OK"
        False si timeout
    """
    print(f"⏳ Attente confirmation SS2 — fin photos roche {label_roche}...")

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(SS2_TIMEOUT)
            s.connect((SS2_IP, SS2_PORT))

            # Envoyer une demande de statut
            demande = json.dumps({
                "type":  "DEMANDE_STATUT",
                "label": int(label_roche),
            }).encode("utf-8")
            s.sendall(demande)

            reponse_raw = s.recv(1024)
            reponse = json.loads(reponse_raw.decode("utf-8"))

            if reponse.get("status") == "OK":
                nb = reponse.get("nb_photos_prises", "?")
                print(f"✅ SS2 — {nb} photos prises pour roche {label_roche}")
                return True
            else:
                print(f"⚠️  SS2 statut inattendu après orbite : {reponse}")
                return False

    except socket.timeout:
        print(f"⚠️  Timeout en attente de fin photo roche {label_roche}")
        return False
    except Exception as e:
        print(f"⚠️  Erreur attente fin photo : {e}")
        return False