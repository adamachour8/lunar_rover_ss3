import socket
import json

HOST = "0.0.0.0"   # écouter sur toutes les interfaces
PORT = 5005

def ajuster_camera(hauteur_cm: float):
    """
    Adapter la caméra selon la hauteur de la roche
    """
    if hauteur_cm > 40:
        print("📹 Caméra descend fortement")
    elif hauteur_cm > 25:
        print("📹 Caméra descend légèrement")
    else:
        print("📹 Caméra position normale")

print("🟢 Serveur prêt — en attente de signaux roche...")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    while True:
        conn, addr = s.accept()
        with conn:
            print(f"\n🔗 Connexion depuis {addr}")
            data = conn.recv(1024)
            if not data:
                continue

            message = json.loads(data.decode("utf-8"))
            print("📨 Message reçu :", message)

            if message["type"] == "ROCHE_DETECTEE":
                hauteur_cm = message["hauteur_cm"]
                ajuster_camera(hauteur_cm)

                print(
                    f"✅ Roche {message['label']} | "
                    f"Hauteur {hauteur_cm:.1f} cm"
                )