import os
import numpy as np
import matplotlib.pyplot as plt

if not os.path.exists("outputs"):
    os.makedirs("outputs")

# I use the same kind of map as before
grid = np.zeros((30, 30), dtype=int)

# 0 flat
# 1 rough
# 2 risky, like snow or ice
# 9 obstacle

grid[5:14, 5:12] = 1
grid[16:24, 2:10] = 2

grid[8:25, 18] = 9
grid[20, 18:25] = 9

np.random.seed(7)

i = 0
while i < 35:
    r = np.random.randint(0, 30)
    c = np.random.randint(0, 30)
    grid[r, c] = 9
    i = i + 1

start = (2, 2)
goal = (27, 27)

grid[start] = 0
grid[goal] = 0

# Now I convert terrain type to cost
cost = np.ones((30, 30))

cost[grid == 1] = 3      # rough terrain is harder
cost[grid == 2] = 6      # risky terrain is more expensive
cost[grid == 9] = 999    # obstacle should not be used


# simple distance estimation to the goal
def heuristic(point, goal_point):
    d = abs(point[0] - goal_point[0]) + abs(point[1] - goal_point[1])
    return d


# A* variables
open_list = []
closed_list = []

open_list.append(start)

came_from = {}

g_score = {}
f_score = {}

g_score[start] = 0
f_score[start] = heuristic(start, goal)

found = False

while len(open_list) > 0:

    # find the point in open_list with lowest f_score
    current = open_list[0]
    current_f = f_score[current]

    for p in open_list:
        if f_score[p] < current_f:
            current = p
            current_f = f_score[p]

    if current == goal:
        found = True
        break

    open_list.remove(current)
    closed_list.append(current)

    # 4 possible movements: up, down, left, right
    neighbors = [
        (current[0] - 1, current[1]),
        (current[0] + 1, current[1]),
        (current[0], current[1] - 1),
        (current[0], current[1] + 1)
    ]

    for nb in neighbors:

        row = nb[0]
        col = nb[1]

        # check map limits
        if row < 0 or row >= 30 or col < 0 or col >= 30:
            continue

        # dont go to obstacle
        if cost[row, col] >= 999:
            continue

        # dont check again if already checked
        if nb in closed_list:
            continue

        new_g = g_score[current] + cost[row, col]

        if nb not in open_list:
            open_list.append(nb)
        else:
            if new_g >= g_score[nb]:
                continue

        came_from[nb] = current
        g_score[nb] = new_g
        f_score[nb] = new_g + heuristic(nb, goal)


# reconstruct path
path = []

if found:
    p = goal
    path.append(p)

    while p != start:
        p = came_from[p]
        path.append(p)

    path.reverse()

print("found path:", found)
print("path length:", len(path))

# plot the map and path
plt.figure()
plt.imshow(grid)

plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")

if found:
    path_rows = []
    path_cols = []

    for p in path:
        path_rows.append(p[0])
        path_cols.append(p[1])

    plt.plot(path_cols, path_rows, linewidth=2, label="A star path")

plt.title("Practice 03 - A star path planning")
plt.legend()
plt.savefig("outputs/practice_03_astar_path.png")
plt.show()

print("saved: outputs/practice_03_astar_path.png")