import socket
import json
import time

SERVER_IP = "192.168.1.20"   # IP ETH du Raspberry caméra
PORT = 5005

def envoyer_roche(objet, position_xy):
    message = {
        "type": "ROCHE_DETECTEE",
        "label": objet.label,
        "hauteur_m": objet.hauteur,
        "hauteur_cm": objet.hauteur * 100,
        "position": {
            "x": float(position_xy[0]),
            "y": float(position_xy[1])
        },
        "timestamp": time.time()
    }

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((SERVER_IP, PORT))
        s.sendall(json.dumps(message).encode("utf-8"))

    print(
        f"📡 Signal Ethernet envoyé → "
        f"Roche {objet.label} "
        f"({objet.hauteur*100:.1f} cm)"
    )