from dataclasses import dataclass, field


@dataclass
class EtatRover:
    x: float = 0.0
    y: float = 0.0
    angle_deg: float = 0.0
    nb_waypoints_completes: int = 0
    mission_succes: bool = False
    erreur: str = ""

    def __repr__(self):
        return (
            f"EtatRover(pos=({self.x:.3f}, {self.y:.3f}), "
            f"angle={self.angle_deg:.1f}°, "
            f"waypoints={self.nb_waypoints_completes})"
        )
