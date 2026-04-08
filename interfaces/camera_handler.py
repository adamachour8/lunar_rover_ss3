import socket
import json
import time
import threading

HOST = "0.0.0.0"
PORT = 5005

_lock = threading.Lock()

_etat = {
    "en_cours":         False,
    "label":            None,
    "nb_photos_prises": 0,
    "nb_photos_total":  0,
    "termine":          False,
    "erreur":           None,
}


def ajuster_camera(hauteur_cm):
    if hauteur_cm > 40:
        inclinaison = "forte_descente"
        zoom        = 1.5
    elif hauteur_cm > 25:
        inclinaison = "legere_descente"
        zoom        = 1.2
    else:
        inclinaison = "normal"
        zoom        = 1.0

    print(f"Camera — {inclinaison} (roche {hauteur_cm:.0f}cm)")
    # TODO : envoyer les commandes reelles a la camera (servomoteur, API, etc.)
    return {"inclinaison": inclinaison, "zoom": zoom}


def _thread_prendre_photos(nb_photos, intervalle_s, label):
    with _lock:
        _etat["nb_photos_prises"] = 0
        _etat["termine"]          = False
        _etat["erreur"]           = None

    print(f"Debut session photo — roche {label} — {nb_photos} photos x {intervalle_s:.1f}s")

    for i in range(nb_photos):
        try:
            # TODO : remplacer par l'appel reel a la camera
            # camera.capture(f"roche_{label}_photo_{i:03d}.jpg")
            print(f"Photo {i+1}/{nb_photos} — roche {label}")
            time.sleep(intervalle_s)
            with _lock:
                _etat["nb_photos_prises"] += 1
        except Exception as e:
            with _lock:
                _etat["erreur"] = str(e)
            print(f"Erreur photo {i+1} : {e}")
            break

    with _lock:
        _etat["termine"] = True
        nb_prises = _etat["nb_photos_prises"]
    print(f"Session terminee — {nb_prises}/{nb_photos} photos")


def traiter_message(message, conn):
    global _etat
    type_msg = message.get("type", "")

    if type_msg == "ROCHE_DETECTEE":
        label          = message["label"]
        hauteur_cm     = message["hauteur_cm"]
        nb_photos      = message.get("nb_photos", 30)
        duree_orbite_s = message.get("duree_orbite_s", 60.0)
        intervalle_s   = duree_orbite_s / nb_photos

        print(f"Roche {label} — {hauteur_cm:.1f}cm — orbite {duree_orbite_s:.1f}s — intervalle {intervalle_s:.1f}s")

        reglages = ajuster_camera(hauteur_cm)

        with _lock:
            _etat["en_cours"]        = True
            _etat["label"]           = label
            _etat["nb_photos_total"] = nb_photos
            _etat["termine"]         = False

        threading.Thread(
            target=_thread_prendre_photos,
            args=(nb_photos, intervalle_s, label),
            daemon=True
        ).start()

        reponse = {
            "status":       "READY",
            "label":        label,
            "reglages":     reglages,
            "intervalle_s": round(intervalle_s, 2),
        }
        conn.sendall(json.dumps(reponse).encode("utf-8"))
        print(f"READY envoye a SS3")

    elif type_msg == "DEMANDE_STATUT":
        label = message.get("label")

        with _lock:
            snapshot = _etat.copy()
            if snapshot["termine"] and not snapshot["erreur"]:
                _etat["en_cours"] = False

        if snapshot["erreur"]:
            reponse = {"status": "ERROR", "label": label, "message": snapshot["erreur"]}
        elif snapshot["termine"]:
            reponse = {"status": "OK", "label": label, "nb_photos_prises": snapshot["nb_photos_prises"]}
        else:
            reponse = {
                "status":           "EN_COURS",
                "label":            label,
                "nb_photos_prises": snapshot["nb_photos_prises"],
                "nb_photos_total":  snapshot["nb_photos_total"],
            }

        conn.sendall(json.dumps(reponse).encode("utf-8"))
        print(f"Statut : {reponse['status']} ({snapshot['nb_photos_prises']}/{snapshot['nb_photos_total']} photos)")

    else:
        print(f"Type inconnu : {type_msg}")
        conn.sendall(json.dumps({"status": "ERROR", "message": f"Type inconnu : {type_msg}"}).encode("utf-8"))


if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as serveur:
        serveur.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serveur.bind((HOST, PORT))
        serveur.listen()
        print(f"Serveur SS2 pret sur {HOST}:{PORT}")

        while True:
            conn, addr = serveur.accept()
            with conn:
                print(f"Connexion depuis {addr}")
                try:
                    data = conn.recv(4096)
                    if data:
                        message = json.loads(data.decode("utf-8"))
                        traiter_message(message, conn)
                except json.JSONDecodeError as e:
                    print(f"JSON invalide : {e}")
                except Exception as e:
                    print(f"Erreur : {e}")