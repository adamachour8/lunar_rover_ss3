import socket
import json

HOST = "10.0.0.2"
PORT = 5005

message = {
    "type": "ROCHE_DETECTEE",
    "label": 1,
    "hauteur_cm": 23.4,
    "duree_orbite_s": 47.0,
    "nb_photos": 30
}

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))
client.sendall(json.dumps(message).encode('utf-8'))

reponse = client.recv(1024)
print(f"Réponse : {reponse.decode('utf-8')}")

client.close()