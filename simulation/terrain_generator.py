import numpy as np

def generate_terrain(n_points, n_rocks):
    n = np.random.uniform(-100, 100, n_points)
    e = np.random.uniform(-100, 100, n_points)
    d = np.random.normal(0, 0.1, n_points)
    points_sol = np.column_stack((n ,e, d))

    n_r = np.random.uniform(-100, 100, n_rocks)
    e_r = np.random.uniform(-100, 100, n_rocks)
    d_r = np.random.uniform(-2, 0.1, n_rocks)
    points_terrain = np.column_stack((n_r, e_r, d_r))

    return np.vstack((points_sol, points_terrain))