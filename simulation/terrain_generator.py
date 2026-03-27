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

def generate_lunar_terrain(n_points=2000):
    np.random.seed(42)
    
    x = np.random.uniform(0, 10, n_points)
    y = np.random.uniform(0, 10, n_points)
    z = np.random.normal(0, 0.02, n_points)  # rugosité sol

    # Cratères
    craters = [(3, 3, 1.2, 0.4), (7, 6, 0.8, 0.3), (5, 8, 1.0, 0.35)]
    for cx, cy, radius, depth in craters:
        dist = np.sqrt((x - cx)**2 + (y - cy)**2)
        mask = dist < radius
        z[mask] -= depth * (1 - dist[mask] / radius)

    # Roches avec forme aplatie (superellipse)
    rocks = [(2, 7, 0.4, 0.3), (6, 2, 0.35, 0.25), (8, 8, 0.45, 0.3),
             (4, 5, 0.3, 0.2), (1, 4, 0.38, 0.28)]
    for rx, ry, size, height in rocks:
        dist = np.sqrt((x - rx)**2 + (y - ry)**2)
        # np.maximum évite le négatif sous la racine
        dome = height * np.sqrt(np.maximum(0, 1 - (dist / size)**2))
        z += dome

    return np.column_stack([x, y, z])