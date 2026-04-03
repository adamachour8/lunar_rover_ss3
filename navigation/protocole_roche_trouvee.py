import numpy as np
from typing import List, Tuple

from perception.filtration import ObjetDetecte
from navigation.triangulation import perform_triangulation
from navigation.astar import (
    construire_grille,
    astar,
    ordre_visite_glouton,
    generer_points_orbite,
    orbite_complete_astar,
    monde_vers_grille,
    cellule_libre_proche,
    chemin_vers_monde,
)

from communication.envoyer_roche import envoyer_roche
from config import ORBIT_RADIUS, ORBIT_N_POINTS, ASTAR_SCAN_DISTANCE


def protocole_roche_trouvee(
    points_sol: np.ndarray,
    points_terrain: np.ndarray,
    objets_interet: List[ObjetDetecte],
    obstacles: List[ObjetDetecte],
    position_depart: Tuple[float, float],
):
    if not objets_interet:
        return None

    # ── NavMesh + grille A* ──────────────────────────────────────────────
    points_navmesh = np.vstack([points_sol, points_terrain])
    _, points_utilises, navigable, _ = perform_triangulation(points_navmesh)

    grille, origine_xy, resolution = construire_grille(
        points_utilises, navigable, obstacles + objets_interet
    )

    # ── Ordre de visite ──────────────────────────────────────────────────
    ordre = ordre_visite_glouton(position_depart, objets_interet)

    position = position_depart
    chemins = []
    waypoints_monde = []

    waypoints_monde.append({
        "x": position[0],
        "y": position[1],
        "type": "depart",
        "label": "Départ"
    })

    # =====================================================================
    # BOUCLE MISSION — UNE ROCHE = UNE ITÉRATION
    # =====================================================================
    for obj in ordre:
        cx, cy = obj.centroide[0], obj.centroide[1]

        # ── APPROCHE ────────────────────────────────────────────────────
        pts_orbite = generer_points_orbite((cx, cy), ORBIT_RADIUS, ORBIT_N_POINTS)
        pt_entree = pts_orbite[
            np.argmin([np.linalg.norm(np.array(pt) - np.array(position)) for pt in pts_orbite])
        ]

        cell_depart = cellule_libre_proche(
            grille, *monde_vers_grille(*position, origine_xy, resolution)
        )
        cell_arrivee = cellule_libre_proche(
            grille, *monde_vers_grille(*pt_entree, origine_xy, resolution)
        )

        if cell_depart is None or cell_arrivee is None:
            continue

        ch_approche = astar(grille, cell_depart, cell_arrivee)
        if ch_approche is None:
            continue

        chemins.append(ch_approche)
        for wx, wy in chemin_vers_monde(ch_approche, origine_xy, resolution)[1:]:
            waypoints_monde.append({
                "x": round(wx, 4),
                "y": round(wy, 4),
                "type": "travel",
                "label": f"Approche roche {obj.label}"
            })

        position = pt_entree

        # 🚨 SIGNAL ETHERNET (DEVANT LA ROCHE)
        envoyer_roche(obj, position)

        # ── ORBITE ──────────────────────────────────────────────────────
        ch_orbite, pts_orbite_conf, _ = orbite_complete_astar(
            grille,
            origine_xy,
            resolution,
            (cx, cy),
            position,
            ORBIT_RADIUS,
            ORBIT_N_POINTS
        )

        if ch_orbite:
            chemins.extend(ch_orbite)
            for wx, wy in pts_orbite_conf:
                waypoints_monde.append({
                    "x": round(wx, 4),
                    "y": round(wy, 4),
                    "type": "orbit",
                    "label": f"Orbite roche {obj.label}"
                })
            position = pts_orbite_conf[-1]

        else:
            # fallback scan
            direction = np.array([cx, cy]) - np.array(position)
            dist = np.linalg.norm(direction)
            if dist > ASTAR_SCAN_DISTANCE:
                pt_scan = position + direction / dist * (dist - ASTAR_SCAN_DISTANCE)
                waypoints_monde.append({
                    "x": round(float(pt_scan[0]), 4),
                    "y": round(float(pt_scan[1]), 4),
                    "type": "scan",
                    "label": f"Scan roche {obj.label}"
                })
                position = (pt_scan[0], pt_scan[1])

    # ── RETOUR ───────────────────────────────────────────────────────────
    cell_depart = cellule_libre_proche(
        grille, *monde_vers_grille(*position, origine_xy, resolution)
    )
    cell_arrivee = cellule_libre_proche(
        grille, *monde_vers_grille(*position_depart, origine_xy, resolution)
    )

    if cell_depart and cell_arrivee:
        ch_retour = astar(grille, cell_depart, cell_arrivee)
        if ch_retour:
            chemins.append(ch_retour)
            for wx, wy in chemin_vers_monde(ch_retour, origine_xy, resolution)[1:]:
                waypoints_monde.append({
                    "x": round(wx, 4),
                    "y": round(wy, 4),
                    "type": "return",
                    "label": "Retour départ"
                })

    waypoints_monde.append({
        "x": position_depart[0],
        "y": position_depart[1],
        "type": "arrivee",
        "label": "Arrivée"
    })

    return {
        "chemins": chemins,
        "waypoints_monde": waypoints_monde,
        "ordre": ordre,
    }