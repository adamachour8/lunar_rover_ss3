from dataclasses import dataclass
import numpy as np


@dataclass
class Terrain:
    points_bruts: np.ndarray
    sol: np.ndarray
    terrain_naturel: np.ndarray
    obstacles_pts: np.ndarray

    @property
    def nb_points(self) -> int:
        return len(self.points_bruts)

    @property
    def nb_obstacles(self) -> int:
        return len(self.obstacles_pts)

    def __repr__(self):
        return (
            f"Terrain(total={self.nb_points} pts, "
            f"sol={len(self.sol)}, "
            f"naturel={len(self.terrain_naturel)}, "
            f"obstacles={self.nb_obstacles})"
        )
