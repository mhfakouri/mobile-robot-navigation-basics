import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

if not os.path.exists("outputs"):
    os.makedirs("outputs")

# bigger map than the first example
grid = np.zeros((30, 30), dtype=int)

# 0 flat
# 1 rough
# 2 snow or ice or risky part
# 9 obstacle

# rough area
grid[5:14, 5:12] = 1

# risky area
grid[16:24, 2:10] = 2

# obstacles, like walls or blocked region
grid[8:25, 18] = 9
grid[20, 18:25] = 9

# random small obstacles
np.random.seed(7)

i = 0
while i < 35:
    r = np.random.randint(0, 30)
    c = np.random.randint(0, 30)
    grid[r, c] = 9
    i = i + 1

start = (2, 2)
goal = (27, 27)

# I dont want start or goal to be obstacle by mistake
grid[start] = 0
grid[goal] = 0

print("map size:", grid.shape)
print("start:", start)
print("goal:", goal)

print("flat:", np.sum(grid == 0))
print("rough:", np.sum(grid == 1))
print("risky:", np.sum(grid == 2))
print("obstacle:", np.sum(grid == 9))

plt.figure()
plt.imshow(grid)

plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")

plt.title("Practice 02 - bigger terrain map")
plt.legend()

plt.savefig("outputs/practice_02_terrain_plot.png")
plt.show()

print("plot saved")