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
grid = np.zeros((30, 30), dtype=int)

# Add rough terrain.
# This could represent uneven forest ground.
grid[5:14, 5:12] = 1

# Add high-risk terrain.
# This could represent snow, ice, soft soil, or slippery terrain.
grid[16:24, 2:10] = 2

# Add obstacle structures.
# These could represent trees, rocks, blocked areas, or unsafe regions.
grid[8:25, 18] = 9
grid[20, 18:25] = 9

# Add a few random obstacles.
# The random seed makes the result repeatable.
np.random.seed(7)

for _ in range(35):
    row = np.random.randint(0, 30)
    col = np.random.randint(0, 30)
    grid[row, col] = 9

# Define start and goal.
start = (2, 2)
goal = (27, 27)

# Make sure start and goal are not obstacles.
grid[start] = 0
grid[goal] = 0

print("Grid shape:", grid.shape)
print("Start:", start)
print("Goal:", goal)
print("Flat cells:", np.sum(grid == 0))
print("Rough cells:", np.sum(grid == 1))
print("High-risk cells:", np.sum(grid == 2))
print("Obstacle cells:", np.sum(grid == 9))

# Plot terrain map.
plt.figure()
plt.imshow(grid)
plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")
plt.title("Practice 02 — Terrain Map")
plt.legend()
plt.tight_layout()

plt.savefig(OUTPUT_DIR / "practice_02_terrain_plot.png", dpi=150)
plt.close()

print("Saved plot to outputs/practice_02_terrain_plot.png")