# Lunar Rover SS3 — Guidage, Localisation et Communication

## Description

Sous-système 3 (SS3) du module lunaire développé dans le cadre du projet AER1110 
à Polytechnique Montréal (Hiver 2026), en collaboration avec l'Agence Spatiale Canadienne.


## Flux d'opération général

1. **Préscan** — Le LiDAR effectue un scan initial, RANSAC modélise le terrain
2. **Pathfinding** — Triangulation + D*-Lite génère un chemin vers les objets détectés
3. **Déplacement** — SS4 exécute le chemin reçu via `motor_control.py`
4. **Scan détaillé** — SS2 prend des images pour le modèle 3D à 15m de l'objet
5. **Retour** — Sur signal de batterie faible, D*-Lite recalcule le chemin de retour
