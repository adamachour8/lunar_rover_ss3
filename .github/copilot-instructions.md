# Lunar Rover SS3 — AI Coding Assistant Instructions

## Project Overview

**Lunar Rover SS3** (Subsystem 3) is a **guidance, localization, and communication system** for a lunar rover module developed at Polytechnique Montréal under the AER1110 project in collaboration with the Canadian Space Agency (Winter 2026).

## Core Architecture & Data Flow

### Processing Pipeline
1. **Perception Phase** (`perception/`)
   - **RANSAC** (`ransac.py`): Filters terrain point clouds to identify obstacles (outliers from ground plane)
   - **DBSCAN** (`DBSCAN.py`): Clusters obstacles into distinct objects using epsilon and min_samples thresholds
   - Input: 3D point cloud from LiDAR; Output: obstacle clusters as dictionary `{label: np.array(N,3)}`

2. **Navigation Phase** (`navigation/`)
   - **Triangulation** (`triangulation.py`): Delaunay triangulation on 2D projection (XY plane) with slope filtering
   - **D*-Lite** (`dstar_lite.py`): Path planning using navigable terrain triangles (WIP)
   - Input: obstacle clusters; Output: path waypoints as 3D coordinates

3. **Execution Phase** (`interfaces/`)
   - **Motor Control** (`motor_control.py`): Sends movement commands to SS4 (propulsion subsystem) (WIP)
   - **Camera Handler** (`camera_handler.py`): Coordinates imaging with SS2 at 15m distance from target (WIP)

4. **Data Exchange** (`communication/`)
   - **Ethernet** (`ethernet.py`): Inter-subsystem communication protocol (WIP)

### Key Data Structures
- **Point clouds**: NumPy arrays of shape `(N, 3)` representing [x, y, z] coordinates
- **Obstacle clusters**: Dictionary `{label: np.array(N,3)}` from DBSCAN output
- **Terrain model**: Ground plane equation from RANSAC: `Ax + By + Cz + D = 0`
- **Navigation mesh**: Delaunay triangulation simplices filtered by slope angle threshold

## Critical Parameters & Tuning

Located in `main.py`:
- `THRESHOLD = 0.1`: Maximum distance (meters) from ground plane to classify point as obstacle
- `EPS = 0.3`: Distance threshold for DBSCAN clustering
- `MIN_SAMPLES = 6`: Minimum points to form a valid cluster
- `angle_max = 30` (in `triangulation.py`): Maximum slope angle in degrees for navigable terrain

**Usage Pattern**: Adjust these values when testing with different point cloud datasets (e.g., `NuagePtsTest1-6.csv`).

## Testing & Simulation

- **Test Data**: CSV files in `simulation/` directory contain pre-recorded point clouds from LiDAR scans
- **Terrain Generation**: `simulation/terrain_generator.py` provides synthetic data with craters and rocks
- **Visualization**: All modules use matplotlib for debugging (3D scatter plots with `projection='3d'`)
- **Test File**: `tests/test_terrain.py` (currently empty—extend with pytest/unittest)

## Current Implementation Status

| Module | Status | Notes |
|--------|--------|-------|
| RANSAC perception | ✅ Complete | Returns outliers as navigable obstacles |
| DBSCAN clustering | ✅ Complete | Returns labeled clusters |
| Triangulation | ✅ Complete | Filters by slope angle |
| D*-Lite pathfinding | ⏳ Empty | Needs implementation |
| Motor control | ⏳ Empty | Awaits interface specification |
| Camera handler | ⏳ Empty | Awaits SS2 integration protocol |
| Ethernet comm | ⏳ Empty | Inter-subsystem message format needed |
| Models (Rover, Terrain) | ⏳ Empty | May define state/configuration classes |

## Code Conventions

1. **NumPy Arrays**: All 3D coordinates are `float64` arrays with shape `(N, 3)` for [x, y, z]
2. **Error Handling**: Functions return `None` on failure (e.g., `triangulation.py` line 49) with console error messages
3. **Visualization**: Commented-out plotting code included in development files for manual debugging
4. **French Comments**: Codebase mixes English and French comments—maintain consistency with existing file
5. **Module Imports**: Files use absolute paths via `sys.path.append(os.path.dirname(...))` for cross-module imports

## Key Integration Points

- **RANSAC → DBSCAN**: Pass `obstacles` (outliers) directly; same 3D coordinate system
- **Perception → Navigation**: Cluster dictionary keys are arbitrary labels; pathfinding must handle variable obstacle counts
- **Navigation → Motor Control**: Path format (waypoint sequence, spacing, speed) not yet specified
- **Recovery Logic**: When battery low, D*-Lite must recalculate return path to rover starting location

## Development Workflow

1. **Run perception pipeline**: `python main.py` executes RANSAC + DBSCAN with `NuagePtsTest1-6.csv`
2. **Visualize results**: Matplotlib 3D plot displays clusters
3. **Switch datasets**: Edit `NOM_FICHIER` in `main.py` to test different terrain scenarios
4. **Debug parameters**: Adjust `THRESHOLD`, `EPS`, `MIN_SAMPLES` and re-run
5. **Add unit tests**: Extend `tests/test_terrain.py` with assertions on cluster counts, terrain slope validation

## Dependencies

- `numpy`: Core numerical operations
- `pyransac3d`: RANSAC plane fitting
- `scikit-learn`: DBSCAN clustering (`sklearn.cluster.DBSCAN`)
- `scipy`: Delaunay triangulation (`scipy.spatial.Delaunay`)
- `matplotlib`: 3D visualization
- See `requirements.txt` for full list

## Next Steps for Implementation

1. Implement D*-Lite using navigable simplices from triangulation
2. Define waypoint output format for motor control
3. Establish ethernet protocol with other subsystems
4. Add comprehensive unit tests for edge cases (empty clusters, degenerate terrains)
5. Profile algorithm runtime on representative point clouds (~10k points)
