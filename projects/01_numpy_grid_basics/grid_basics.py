from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Output folder for generated plots.
OUTPUT_DIR = Path(__file__).resolve().parents[2] / "outputs"
OUTPUT_DIR.mkdir(exist_ok=True)

# Terrain labels:
# 0 = flat terrain
# 1 = rough terrain
# 2 = high-risk terrain, for example snow or ice
# 9 = obstacle
grid = np.zeros((10, 10), dtype=int)

# Add simple terrain areas.
grid[2:5, 3:6] = 1
grid[6:8, 1:4] = 2
grid[4:8, 7] = 9

# Start and goal positions.
# Format: row, column
start = (0, 0)
goal = (9, 9)

# Convert terrain labels into movement costs.
cost_map = np.ones_like(grid, dtype=float)
cost_map[grid == 1] = 3.0
cost_map[grid == 2] = 6.0
cost_map[grid == 9] = 999.0

print("Grid shape:", grid.shape)
print("Start:", start)
print("Goal:", goal)
print("Number of rough cells:", np.sum(grid == 1))
print("Number of high-risk cells:", np.sum(grid == 2))
print("Number of obstacle cells:", np.sum(grid == 9))

plt.figure()
plt.imshow(grid)
plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")
plt.title("Practice 01 — Basic Terrain Grid")
plt.legend()
plt.tight_layout()

plt.savefig(OUTPUT_DIR / "practice_01_grid_basics.png", dpi=150)
plt.close()

print("Saved plot to outputs/practice_01_grid_basics.png")