import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    def perform_triangulation(points, angle_max=30):
        """
        Perform 2D Delaunay triangulation projected on XY plane.

        Args:
            points (np.array) : Tous les points du terrain (sol + obstacles), shape (N, 3)
            angle_max (float) : Angle max de pente en degrés. Défaut 30°.

        Returns:
            tri           : Objet Delaunay
            navigable     : Array de simplices navigables (angle < angle_max)
            non_navigable : Array de simplices non-navigables (angle >= angle_max)
        """
        try:
            points_3d = np.array(points, dtype=float)

            if points_3d.ndim != 2 or points_3d.shape[1] != 3:
                raise ValueError("Points must be a 2D array with shape (n, 3).")

            if len(points_3d) < 3:
                raise ValueError("At least 3 points are required for triangulation.")

            points_2d = points_3d[:, :2]
            tri = Delaunay(points_2d)

            navigable = []
            non_navigable = []

            for simplex in tri.simplices:
                p1, p2, p3 = points_3d[simplex]

                v1 = p2 - p1
                v2 = p3 - p1
                normale = np.cross(v1, v2)
                norme = np.linalg.norm(normale)

                if norme == 0:
                    continue

                normale = normale / norme
                angle = np.degrees(np.arccos(np.clip(abs(normale[2]), 0, 1)))

                if angle < angle_max:
                    navigable.append(simplex)
                else:
                    non_navigable.append(simplex)

            return tri, np.array(navigable), np.array(non_navigable)

        except Exception as e:
            print(f"Error during triangulation: {e}")
            return None, None, None


    def plot_triangulation(points, navigable, non_navigable):
        """
        Plot navigable and non-navigable triangles in 3D.

        Args:
            points        : Tous les points du terrain (sol + obstacles), shape (N, 3)
            navigable     : Array de simplices navigables
            non_navigable : Array de simplices non-navigables
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        points_3d = np.array(points, dtype=float)

        for simplex in navigable:
            triangle = points_3d[simplex]
            triangle = np.vstack([triangle, triangle[0]])
            ax.plot(triangle[:, 0], triangle[:, 1], triangle[:, 2],
                    color='green', alpha=0.3)

        for simplex in non_navigable:
            triangle = points_3d[simplex]
            triangle = np.vstack([triangle, triangle[0]])
            ax.plot(triangle[:, 0], triangle[:, 1], triangle[:, 2],
                    color='red', alpha=0.5)

        ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2],
                color='black', s=5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title("Triangulation 3D")
        plt.show()