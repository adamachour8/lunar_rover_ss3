import socket
import json
import time
import threading

HOST = "0.0.0.0"   # Écouter sur toutes les interfaces Ethernet
PORT = 5005

# État interne — partagé entre threads (connexion SS3 + thread photo)
_etat = {
    "en_cours":        False,
    "label":           None,
    "nb_photos_prises": 0,
    "nb_photos_total":  0,
    "termine":         False,
    "erreur":          None,
}


# ===========================================================================
# RÉGLAGE CAMÉRA
# ===========================================================================

def ajuster_camera(hauteur_cm):
    """
    Règle l'inclinaison et le zoom de la caméra selon la hauteur de la roche.
    À adapter selon les specs réelles de la caméra SS2.

    Returns:
        dict avec les réglages appliqués
    """
    if hauteur_cm > 40:
        inclinaison = "forte_descente"
        zoom        = 1.5
        print(f"📹 Caméra — descente forte (roche {hauteur_cm:.0f}cm)")
    elif hauteur_cm > 25:
        inclinaison = "legere_descente"
        zoom        = 1.2
        print(f"📹 Caméra — descente légère (roche {hauteur_cm:.0f}cm)")
    else:
        inclinaison = "normal"
        zoom        = 1.0
        print(f"📹 Caméra — position normale (roche {hauteur_cm:.0f}cm)")

    # TODO : envoyer les commandes réelles à la caméra (servomoteur, API caméra, etc.)
    return {"inclinaison": inclinaison, "zoom": zoom}


# ===========================================================================
# THREAD PHOTO — tourne en parallèle de l'orbite du rover
# ===========================================================================

def _thread_prendre_photos(nb_photos, intervalle_s, label):
    """
    Prend nb_photos photos, espacées de intervalle_s secondes.
    Tourne dans un thread séparé pendant que le rover fait son orbite.
    """
    global _etat
    _etat["nb_photos_prises"] = 0
    _etat["termine"]          = False
    _etat["erreur"]           = None

    print(f"📸 Début session photo — roche {label} — "
          f"{nb_photos} photos × {intervalle_s:.1f}s = {nb_photos*intervalle_s:.0f}s")

    for i in range(nb_photos):
        try:
            # TODO : remplacer par l'appel réel à la caméra
            # Exemple : camera.capture(f"roche_{label}_photo_{i:03d}.jpg")
            print(f"  📷 Photo {i+1}/{nb_photos} — roche {label}")
            time.sleep(intervalle_s)
            _etat["nb_photos_prises"] += 1

        except Exception as e:
            _etat["erreur"] = str(e)
            print(f"  ❌ Erreur photo {i+1} : {e}")
            break

    _etat["termine"] = True
    print(f"✅ Session photo terminée — {_etat['nb_photos_prises']}/{nb_photos} photos")


# ===========================================================================
# SERVEUR TCP PRINCIPAL
# ===========================================================================

def traiter_message(message, conn):
    """
    Traite un message reçu de SS3 et envoie la réponse appropriée.
    """
    global _etat
    type_msg = message.get("type", "")

    # ── ÉTAPE 1 : Roche détectée — préparer la caméra ─────────────────────
    if type_msg == "ROCHE_DETECTEE":
        label          = message["label"]
        hauteur_cm     = message["hauteur_cm"]
        nb_photos      = message.get("nb_photos", 30)
        duree_orbite_s = message.get("duree_orbite_s", 60.0)
        intervalle_s   = duree_orbite_s / nb_photos

        print(f"\n🔔 Roche {label} détectée — {hauteur_cm:.1f}cm — "
              f"orbite {duree_orbite_s:.1f}s — "
              f"intervalle photo {intervalle_s:.1f}s")

        # Régler la caméra
        reglages = ajuster_camera(hauteur_cm)

        # Mettre à jour l'état interne
        _etat["en_cours"]        = True
        _etat["label"]           = label
        _etat["nb_photos_total"] = nb_photos
        _etat["termine"]         = False

        # Lancer le thread photo (parallèle au rover)
        t = threading.Thread(
            target=_thread_prendre_photos,
            args=(nb_photos, intervalle_s, label),
            daemon=True
        )
        t.start()

        # Confirmer à SS3 que la caméra est prête → SS3 peut démarrer l'orbite
        reponse = {
            "status":      "READY",
            "label":       label,
            "reglages":    reglages,
            "intervalle_s": round(intervalle_s, 2),
        }
        conn.sendall(json.dumps(reponse).encode("utf-8"))
        print(f"📤 READY envoyé à SS3 — orbite peut commencer")

    # ── ÉTAPE 3 : SS3 demande si les photos sont terminées ────────────────
    elif type_msg == "DEMANDE_STATUT":
        label = message.get("label")

        if _etat["erreur"]:
            reponse = {
                "status":  "ERROR",
                "label":   label,
                "message": _etat["erreur"],
            }
        elif _etat["termine"]:
            reponse = {
                "status":          "OK",
                "label":           label,
                "nb_photos_prises": _etat["nb_photos_prises"],
            }
            _etat["en_cours"] = False
        else:
            # Photos pas encore terminées — SS3 peut attendre ou continuer
            reponse = {
                "status":          "EN_COURS",
                "label":           label,
                "nb_photos_prises": _etat["nb_photos_prises"],
                "nb_photos_total":  _etat["nb_photos_total"],
            }

        conn.sendall(json.dumps(reponse).encode("utf-8"))
        print(f"📤 Statut envoyé à SS3 : {reponse['status']} "
              f"({_etat['nb_photos_prises']}/{_etat['nb_photos_total']} photos)")

    else:
        print(f"⚠️  Type de message inconnu : {type_msg}")
        reponse = {"status": "ERROR", "message": f"Type inconnu : {type_msg}"}
        conn.sendall(json.dumps(reponse).encode("utf-8"))


# ===========================================================================
# POINT D'ENTRÉE
# ===========================================================================

if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as serveur:
        serveur.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serveur.bind((HOST, PORT))
        serveur.listen()
        print(f"🟢 Serveur SS2 prêt sur {HOST}:{PORT} — en attente de SS3...")

        while True:
            conn, addr = serveur.accept()
            with conn:
                print(f"\n🔗 Connexion depuis {addr}")
                try:
                    data = conn.recv(4096)
                    if data:
                        message = json.loads(data.decode("utf-8"))
                        traiter_message(message, conn)
                except json.JSONDecodeError as e:
                    print(f"❌ JSON invalide reçu : {e}")
                except Exception as e:
                    print(f"❌ Erreur traitement message : {e}")