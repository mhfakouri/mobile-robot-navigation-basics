import os
import math
import numpy as np
import matplotlib.pyplot as plt

# finding output folder
this_file = os.path.abspath(__file__)
this_folder = os.path.dirname(this_file)
main_folder = os.path.dirname(os.path.dirname(this_folder))
output_folder = os.path.join(main_folder, "outputs")

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

print("output folder:", output_folder)

# -----------------------------
# make terrain map
# -----------------------------

grid = np.zeros((30, 30), dtype=int)

# 0 flat
# 1 rough
# 2 risky terrain, like snow or ice
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

# terrain cost
cost = np.ones((30, 30))
cost[grid == 1] = 3
cost[grid == 2] = 6
cost[grid == 9] = 999


# -----------------------------
# A star path planning
# -----------------------------

def h(p, g):
    return abs(p[0] - g[0]) + abs(p[1] - g[1])


open_list = []
closed_list = []

open_list.append(start)

came_from = {}

g_score = {}
f_score = {}

g_score[start] = 0
f_score[start] = h(start, goal)

found = False

while len(open_list) > 0:

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

    neighbors = [
        (current[0] - 1, current[1]),
        (current[0] + 1, current[1]),
        (current[0], current[1] - 1),
        (current[0], current[1] + 1)
    ]

    for nb in neighbors:

        row = nb[0]
        col = nb[1]

        if row < 0 or row >= 30 or col < 0 or col >= 30:
            continue

        if cost[row, col] >= 999:
            continue

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
        f_score[nb] = new_g + h(nb, goal)


path = []

if found:
    p = goal
    path.append(p)

    while p != start:
        p = came_from[p]
        path.append(p)

    path.reverse()

print("path found:", found)
print("path length:", len(path))


# -----------------------------
# convert A star path to waypoints
# -----------------------------

# A* path is row, column.
# For robot simulation I use x, y.
# x = column
# y = row

waypoints = []

if found:
    j = 0
    while j < len(path):
        row = path[j][0]
        col = path[j][1]

        waypoints.append([col, row])

        # I dont need every cell as waypoint
        # this makes the path easier for the simple robot
        j = j + 3

    # make sure goal is included
    last_row = path[-1][0]
    last_col = path[-1][1]
    waypoints.append([last_col, last_row])

waypoints = np.array(waypoints, dtype=float)

print("number of waypoints:", len(waypoints))


# -----------------------------
# simple vehicle tracking
# -----------------------------

x = waypoints[0, 0]
y = waypoints[0, 1]
theta = 0.0

dt = 0.1
v = 1.0
k = 2.0

target_index = 1

x_list = []
y_list = []

i = 0

while i < 2000:

    target_x = waypoints[target_index, 0]
    target_y = waypoints[target_index, 1]

    dx = target_x - x
    dy = target_y - y

    dist = math.sqrt(dx*dx + dy*dy)

    if dist < 0.4:
        if target_index < len(waypoints) - 1:
            target_index = target_index + 1
        else:
            break

    target_x = waypoints[target_index, 0]
    target_y = waypoints[target_index, 1]

    dx = target_x - x
    dy = target_y - y

    desired_theta = math.atan2(dy, dx)

    theta_error = desired_theta - theta
    theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

    omega = k * theta_error

    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt
    theta = theta + omega * dt

    x_list.append(x)
    y_list.append(y)

    i = i + 1

print("tracking steps:", len(x_list))
print("final x:", x)
print("final y:", y)


# -----------------------------
# plot result
# -----------------------------

plt.figure()
plt.imshow(grid)

# plot A star path
if found:
    path_rows = []
    path_cols = []

    for p in path:
        path_rows.append(p[0])
        path_cols.append(p[1])

    plt.plot(path_cols, path_rows, linewidth=2, label="A star path")

# plot vehicle trajectory
plt.plot(x_list, y_list, linewidth=2, label="vehicle tracking")

# plot waypoints
plt.scatter(waypoints[:, 0], waypoints[:, 1], marker=".", label="waypoints")

plt.scatter(start[1], start[0], marker="o", label="start")
plt.scatter(goal[1], goal[0], marker="x", label="goal")

plt.title("Practice 05 - A star path with vehicle tracking")
plt.legend(loc="lower right")

save_name = os.path.join(output_folder, "practice_05_astar_vehicle_tracking.png")
plt.savefig(save_name)
plt.show()

print("saved:", save_name)