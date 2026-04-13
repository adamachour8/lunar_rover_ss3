"""Utilitaires serie partages : envoi de commandes et auto-detection des Arduinos."""
import time
import serial
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (PORTS_CANDIDATS, ARDUINO_BAUDRATE, ARDUINO_TIMEOUT,
                    PONG_MOTEUR, PONG_CAM)


def envoyer_commande(arduino, commande):
    """
    Envoie une commande a un Arduino et attend la reponse.
    Retourne 'D', 'PRET', 'PONG_*' ou None si timeout.
    Les lignes commencant par '#' sont du debug et sont ignorees.
    """
    arduino.write(f"{commande}\n".encode('utf-8'))
    while True:
        ligne = arduino.readline().decode('utf-8', errors='ignore').strip()
        if not ligne:
            print(f"  Timeout sur commande : {commande}")
            return None
        if ligne.startswith("#"):
            print(f"  [Arduino debug] {ligne}")
            continue
        if ligne in ("D", "PRET", PONG_MOTEUR, PONG_CAM) or ligne.startswith("PONG"):
            print(f"  Envoye : {commande} -> Recu : {ligne}")
            return ligne
        print(f"  [Arduino] {ligne}")


def detecter_arduinos():
    """
    Ouvre chaque port candidat, envoie PING, identifie via la reponse.
    Retourne (arduino_moteur, arduino_cam). L'un ou l'autre peut etre None.
    """
    arduino_moteur = None
    arduino_cam    = None

    for port in PORTS_CANDIDATS:
        try:
            ser = serial.Serial(port, baudrate=ARDUINO_BAUDRATE, timeout=ARDUINO_TIMEOUT)
        except (serial.SerialException, OSError):
            continue

        time.sleep(2)  # reset auto de l'Arduino a l'ouverture du port (DTR)
        ser.reset_input_buffer()

        while ser.in_waiting:
            ligne = ser.readline().decode('utf-8', errors='ignore').strip()
            if ligne:
                print(f"[{port} init] {ligne}")

        ser.write(b"PING\n")

        reponse = None
        for _ in range(5):
            ligne = ser.readline().decode('utf-8', errors='ignore').strip()
            if ligne and not ligne.startswith("#"):
                reponse = ligne
                break

        print(f"[{port}] PING -> {reponse}")

        if reponse == PONG_MOTEUR and arduino_moteur is None:
            arduino_moteur = ser
            print(f"  -> Arduino MOTEUR sur {port}")
        elif reponse == PONG_CAM and arduino_cam is None:
            arduino_cam = ser
            print(f"  -> Arduino CAM SS2 sur {port}")
        else:
            ser.close()

        if arduino_moteur and arduino_cam:
            break

    return arduino_moteur, arduino_cam