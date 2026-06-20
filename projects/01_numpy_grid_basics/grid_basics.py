import os
import numpy as np
import matplotlib.pyplot as plt

# this is the output folder
if not os.path.exists("outputs"):
    os.makedirs("outputs")

# 0 means normal ground
# 1 means rough ground
# 2 means risky ground like snow or ice
# 9 means obstacle
grid = np.zeros((10, 10), dtype=int)

# putting some areas manually
grid[2:5, 3:6] = 1
grid[6:8, 1:4] = 2
grid[4:8, 7] = 9

start = (0, 0)
goal = (9, 9)

# now I make another grid for cost
cost = np.ones((10, 10))

cost[grid == 1] = 3
cost[grid == 2] = 6
cost[grid == 9] = 999

print("grid size is:", grid.shape)
print("start point:", start)
print("goal point:", goal)

print("rough cells:", np.sum(grid == 1))
print("risky cells:", np.sum(grid == 2))
print("obstacle cells:", np.sum(grid == 9))

plt.figure()
plt.imshow(grid)

# scatter uses x,y but my points are row,col, so I put [1] first
plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")

plt.title("Practice 01 - simple terrain grid")
plt.legend()
plt.savefig("outputs/practice_01_grid_basics1.png")
plt.show()

print("done")