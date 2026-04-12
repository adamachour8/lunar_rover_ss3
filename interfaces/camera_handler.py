import socket
import json
import time
import threading

HOST = "0.0.0.0"
PORT = 5005


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


def prendre_photos(nb_photos, intervalle_s, label):
    print(f"Debut session photo — roche {label} — {nb_photos} photos x {intervalle_s:.1f}s")

    for i in range(nb_photos):
        try:
            # TODO : remplacer par l'appel reel a la camera
            # camera.capture(f"roche_{label}_photo_{i:03d}.jpg")
            print(f"Photo {i+1}/{nb_photos} — roche {label}")
            time.sleep(intervalle_s)
        except Exception as e:
            print(f"Erreur photo {i+1} : {e}")
            break

    print(f"Session terminee — roche {label}")


def traiter_message(message):
    type_msg = message.get("type", "")

    if type_msg == "ROCHE_DETECTEE":
        label          = message["label"]
        hauteur_cm     = message["hauteur_cm"]
        nb_photos      = message.get("nb_photos", 30)
        duree_orbite_s = message.get("duree_orbite_s", 60.0)
        intervalle_s   = duree_orbite_s / nb_photos

        print(f"Roche {label} — {hauteur_cm:.1f}cm — orbite {duree_orbite_s:.1f}s — intervalle {intervalle_s:.1f}s")

        ajuster_camera(hauteur_cm)

        # Lancement en thread pour ne pas bloquer le serveur
        threading.Thread(
            target=prendre_photos,
            args=(nb_photos, intervalle_s, label),
            daemon=True
        ).start()

    else:
        print(f"Type inconnu : {type_msg}")


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
                        traiter_message(message)
                except json.JSONDecodeError as e:
                    print(f"JSON invalide : {e}")
                except Exception as e:
                    print(f"Erreur : {e}")