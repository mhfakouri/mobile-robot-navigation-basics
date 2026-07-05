# Mobile Robot Navigation: A* Planning, Risk-Aware Routing, and Disturbed Tracking

This Python project demonstrates a compact mobile-robot navigation pipeline connecting terrain-grid representation, terrain-dependent path costs, A* planning, waypoint generation, unicycle-model trajectory tracking, disturbance injection, and quantitative performance evaluation.

The final comparison contrasts a distance-focused route with a risk-aware route using path length, terrain cost, risk exposure, tracking error, and navigation success. The project is intentionally simulation-based and uses manually defined maps and terrain costs; it does not represent a complete autonomous navigation stack.

## Pipeline

```text
terrain map → terrain cost → A* path → risk-aware route → waypoints → vehicle tracking → performance metrics
```

## Main Features

- NumPy-based terrain-grid generation
- Flat, rough, risky, and obstacle cell types
- Four-connected A* path planning
- Terrain-dependent planning costs
- Unicycle-model waypoint tracking
- Heading disturbance and terrain-dependent speed reduction
- Path cost and risk-exposure calculation
- Mean and maximum tracking-error calculation
- Shortest-path versus risk-aware planning comparison

## Requirements

```text
numpy
matplotlib
```

Install the dependencies with:

```bash
pip install -r requirements.txt
```

## Repository Structure

```text
projects/
├── 01_numpy_grid_basics/
├── 02_terrain_plotting/
├── 03_astar_path_planning/
├── 04_vehicle_tracking/
├── 05_astar_vehicle_tracking/
├── 06_disturbance_and_metrics/
└── 07_shortest_vs_risk_aware_planning/
outputs/
requirements.txt
README.md
```

## 1. Terrain-Grid Representation

The first examples introduce a small grid and then a larger 30 × 30 terrain map. Each cell is assigned a terrain category:

```text
0 = flat terrain
1 = rough terrain
2 = risky terrain, such as snow or ice
9 = obstacle
```

The terrain labels are manually specified and are not derived from sensor measurements.

![Basic terrain grid](outputs/practice_01_grid_basics.png)

![Larger terrain map](outputs/practice_02_terrain_plot.png)

Run:

```bash
python projects/01_numpy_grid_basics/grid_basics.py
python projects/02_terrain_plotting/terrain_plot.py
```

## 2. A* Path Planning

The planner searches for a collision-free route from a start cell to a goal cell. Movement is limited to the four cardinal directions. Terrain costs allow rough and risky areas to be penalized more heavily than flat terrain.

![A* path](outputs/practice_03_astar_path.png)

Run:

```bash
python projects/03_astar_path_planning/astar_demo.py
```

## 3. Unicycle-Model Tracking

A simplified vehicle model with state `(x, y, theta)` tracks a sequence of waypoints. The model omits detailed wheel, actuator, slip, and contact dynamics, but provides a transparent example of connecting planned grid paths to closed-loop motion.

![Vehicle tracking](outputs/practice_04_vehicle_tracking.png)

Run:

```bash
python projects/04_vehicle_tracking/vehicle_tracking.py
```

## 4. A* Path and Vehicle Tracking

Grid cells are converted from `(row, column)` coordinates to Cartesian-style `(x, y)` waypoints. A subset of path cells is used to reduce excessive turning in the simplified tracker.

![A* path and vehicle tracking](outputs/practice_05_astar_vehicle_tracking.png)

Run:

```bash
python projects/05_astar_vehicle_tracking/astar_vehicle_tracking.py
```

## 5. Disturbance and Tracking Metrics

The disturbed-tracking example adds bounded random heading noise and terrain-dependent speed reduction. It reports:

- A* path length
- Number of selected waypoints
- Total path cost
- Rough and risky cells traversed
- Mean tracking error
- Maximum tracking error

A representative run produced:

```text
A* path cells: 51
selected waypoints: 18
path cost: 51.0
rough cells in path: 0
risky cells in path: 0
mean tracking error: 1.71
max tracking error: 3.30
```

These values are specific to the included map, random seeds, controller parameters, and simplified error definition.

![Disturbed tracking](outputs/practice_06_disturbed_tracking.png)

![Tracking error](outputs/practice_06_tracking_error.png)

Run:

```bash
python projects/06_disturbance_and_metrics/disturbance_and_metrics.py
```

## 6. Shortest-Path versus Risk-Aware Planning

The final example compares two A* cost configurations on the same map:

- **Distance-focused planning:** prioritizes a shorter route and may cross risky terrain.
- **Risk-aware planning:** assigns larger costs to rough and risky cells, allowing a longer but lower-risk route.

![Shortest path](outputs/planned_path_shortest.png)

![Risk-aware path](outputs/planned_path_risk_aware.png)

![Planner comparison](outputs/planner_comparison_same_map.png)

| Planner | Path length | Total terrain cost | Risk exposure | Tracking error | Success |
|---|---:|---:|---:|---:|---|
| Distance-focused path | 25 | 179.0 | 14 | 1.81 | Yes |
| Risk-aware path | 33 | 33.0 | 0 | 1.71 | Yes |

For this specific scenario, the risk-aware route is longer but avoids the risky cells crossed by the distance-focused route.

![Tracking comparison](outputs/planner_tracking_comparison.png)

Run:

```bash
python projects/07_shortest_vs_risk_aware_planning/shortest_vs_risk_aware.py
```

## Reproducibility Notes

- Random seeds are fixed in the included examples where random obstacles or disturbances are generated.
- The maps and terrain costs are manually defined.
- Reported metrics are demonstration results, not guarantees for other maps or parameter settings.
- The tracking error is calculated relative to the active waypoint in the simplified controller.

## Limitations

- No ROS 2 integration in this repository
- No real robot or sensor data
- No LiDAR, camera, localization, or mapping subsystem
- No kinodynamic or continuous-curvature planning
- No wheel-slip or actuator model
- No formal safety guarantee
- Manually selected terrain categories and cost weights

## Possible Extensions

- Integrate the planner and tracker with ROS 2
- Add diagonal motion and alternative heuristics
- Compare A* with Dijkstra, D* Lite, or sampling-based planners
- Add continuous path smoothing and curvature constraints
- Use real or procedurally generated elevation and traversability maps
- Evaluate multiple random seeds and maps
- Introduce model-predictive or robust trajectory tracking

## Author

Mohammad Hossein Fakouri  
M.Sc. Mechanical Engineering – Applied Design  
Research interests: robotics, control under uncertainty, learning-based control, and robot simulation
