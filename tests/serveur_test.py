import socket
import json

HOST = "0.0.0.0"
PORT = 5005

serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serveur.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
serveur.bind((HOST, PORT))
serveur.listen(1)

print(f"En attente sur le port {PORT}...")

conn, adresse = serveur.accept()
print(f"Connexion reçue de {adresse}")

data = conn.recv(1024)
message = json.loads(data.decode('utf-8'))
print(f"Type : {message['type']}")
print(f"Roche {message['label']} — {message['hauteur_cm']}cm")

reponse = {"status": "READY"}
conn.sendall(json.dumps(reponse).encode('utf-8'))
conn.close()