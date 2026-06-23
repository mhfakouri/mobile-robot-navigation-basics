import os
import math
import heapq

import numpy as np
import matplotlib.pyplot as plt


# Practice 07
# Compare shortest path and risk-aware path on the same terrain map.
# This is a small practice script, not a full robotics simulator.


def make_output_folder():
    current_file = os.path.abspath(__file__)
    current_folder = os.path.dirname(current_file)
    repo_folder = os.path.dirname(os.path.dirname(current_folder))
    output_folder = os.path.join(repo_folder, "outputs")

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    return output_folder


def make_grid_map():
    # map codes:
    # 0 = flat terrain
    # 1 = rough terrain
    # 2 = risky terrain
    # 9 = obstacle

    grid = np.zeros((30, 30), dtype=int)

    # risky area in the middle of the map
    grid[12:19, 8:22] = 2

    # some rough areas
    grid[5:9, 12:20] = 1
    grid[21:25, 6:15] = 1
    grid[4:8, 23:27] = 1

    # obstacles
    grid[2:11, 6] = 9
    grid[20:28, 23] = 9
    grid[9, 17:23] = 9
    grid[25, 12:18] = 9

    start = (15, 2)    # row, column
    goal = (15, 27)    # row, column

    grid[start] = 0
    grid[goal] = 0

    return grid, start, goal


def make_cost_map(grid, planner_name):
    cost = np.ones(grid.shape)

    if planner_name == "shortest":
        cost[grid == 1] = 1.0
        cost[grid == 2] = 1.0

    if planner_name == "risk_aware":
        cost[grid == 1] = 3.0
        cost[grid == 2] = 12.0

    cost[grid == 9] = 999.0

    return cost


def heuristic(point, goal):
    # Manhattan distance. This works because the robot moves up, down, left, right.
    row_difference = abs(point[0] - goal[0])
    col_difference = abs(point[1] - goal[1])
    return row_difference + col_difference


def get_neighbors(point):
    row = point[0]
    col = point[1]

    neighbors = []
    neighbors.append((row - 1, col))
    neighbors.append((row + 1, col))
    neighbors.append((row, col - 1))
    neighbors.append((row, col + 1))

    return neighbors


def astar(grid, cost, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {}
    g_score[start] = 0.0

    visited = set()

    while len(open_list) > 0:
        current_item = heapq.heappop(open_list)
        current = current_item[1]

        if current in visited:
            continue

        if current == goal:
            break

        visited.add(current)

        neighbors = get_neighbors(current)

        for neighbor in neighbors:
            row = neighbor[0]
            col = neighbor[1]

            if row < 0 or row >= grid.shape[0]:
                continue
            if col < 0 or col >= grid.shape[1]:
                continue
            if cost[row, col] >= 999:
                continue

            new_cost = g_score[current] + cost[row, col]

            if neighbor not in g_score or new_cost < g_score[neighbor]:
                g_score[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(open_list, (priority, neighbor))
                came_from[neighbor] = current

    path = []

    if goal not in came_from:
        return path

    current = goal
    path.append(current)

    while current != start:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def path_rows_cols(path):
    rows = []
    cols = []

    for point in path:
        rows.append(point[0])
        cols.append(point[1])

    return rows, cols


def calculate_path_metrics(path, grid, risk_aware_cost):
    path_length = len(path) - 1
    total_terrain_cost = 0.0
    risk_exposure = 0
    rough_cells = 0

    for point in path[1:]:
        row = point[0]
        col = point[1]

        total_terrain_cost = total_terrain_cost + risk_aware_cost[row, col]

        if grid[row, col] == 2:
            risk_exposure = risk_exposure + 1

        if grid[row, col] == 1:
            rough_cells = rough_cells + 1

    return path_length, total_terrain_cost, risk_exposure, rough_cells


def path_to_waypoints(path):
    waypoints = []

    for i in range(0, len(path), 3):
        row = path[i][0]
        col = path[i][1]
        waypoints.append([col, row])

    last_row = path[-1][0]
    last_col = path[-1][1]

    if waypoints[-1] != [last_col, last_row]:
        waypoints.append([last_col, last_row])

    return np.array(waypoints)


def simple_tracking(waypoints, grid):
    # This is only a simple unicycle-like tracking test.
    # It is not meant to be a real vehicle model.

    x = waypoints[0, 0]
    y = waypoints[0, 1]
    theta = 0.0

    target_index = 1
    dt = 0.1
    speed = 1.0
    turn_gain = 2.0

    x_history = []
    y_history = []
    error_history = []

    np.random.seed(4)

    for step in range(2500):
        target_x = waypoints[target_index, 0]
        target_y = waypoints[target_index, 1]

        dx = target_x - x
        dy = target_y - y
        distance_error = math.sqrt(dx * dx + dy * dy)
        error_history.append(distance_error)

        if distance_error < 0.4:
            if target_index < len(waypoints) - 1:
                target_index = target_index + 1
            else:
                break

        target_x = waypoints[target_index, 0]
        target_y = waypoints[target_index, 1]

        desired_theta = math.atan2(target_y - y, target_x - x)
        theta_error = desired_theta - theta
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        row = int(round(y))
        col = int(round(x))

        row = max(0, min(row, grid.shape[0] - 1))
        col = max(0, min(col, grid.shape[1] - 1))

        terrain = grid[row, col]

        current_speed = speed
        noise_level = 0.04

        if terrain == 1:
            current_speed = 0.8
            noise_level = 0.08

        if terrain == 2:
            current_speed = 0.55
            noise_level = 0.14

        noise = np.random.normal(0.0, noise_level)

        omega = turn_gain * theta_error + noise

        x = x + current_speed * math.cos(theta) * dt
        y = y + current_speed * math.sin(theta) * dt
        theta = theta + omega * dt

        x_history.append(x)
        y_history.append(y)

    mean_error = float(np.mean(error_history))
    max_error = float(np.max(error_history))

    final_dx = waypoints[-1, 0] - x
    final_dy = waypoints[-1, 1] - y
    final_error = math.sqrt(final_dx * final_dx + final_dy * final_dy)

    success = final_error < 1.0

    return x_history, y_history, mean_error, max_error, final_error, success


def draw_one_path(grid, start, goal, path, title, file_name):
    rows, cols = path_rows_cols(path)

    plt.figure(figsize=(7, 6))
    plt.imshow(grid)
    plt.plot(cols, rows, linewidth=2)
    plt.scatter(start[1], start[0], marker="o", label="start")
    plt.scatter(goal[1], goal[0], marker="x", label="goal")
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(file_name, dpi=160)
    plt.close()


def draw_two_paths(grid, start, goal, shortest_path, risk_path, file_name):
    short_rows, short_cols = path_rows_cols(shortest_path)
    risk_rows, risk_cols = path_rows_cols(risk_path)

    plt.figure(figsize=(7, 6))
    plt.imshow(grid)
    plt.plot(short_cols, short_rows, linewidth=2, label="shortest path")
    plt.plot(risk_cols, risk_rows, linewidth=2, label="risk-aware path")
    plt.scatter(start[1], start[0], marker="o", label="start")
    plt.scatter(goal[1], goal[0], marker="x", label="goal")
    plt.title("Shortest path vs risk-aware path")
    plt.legend()
    plt.tight_layout()
    plt.savefig(file_name, dpi=160)
    plt.close()


def draw_tracking(grid, shortest_path, risk_path, shortest_track, risk_track, file_name):
    short_rows, short_cols = path_rows_cols(shortest_path)
    risk_rows, risk_cols = path_rows_cols(risk_path)

    plt.figure(figsize=(7, 6))
    plt.imshow(grid)
    plt.plot(short_cols, short_rows, "--", linewidth=2, label="shortest planned")
    plt.plot(shortest_track[0], shortest_track[1], linewidth=2, label="shortest tracked")
    plt.plot(risk_cols, risk_rows, "--", linewidth=2, label="risk-aware planned")
    plt.plot(risk_track[0], risk_track[1], linewidth=2, label="risk-aware tracked")
    plt.title("Simple tracking comparison")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(file_name, dpi=160)
    plt.close()


def write_table(file_name, rows):
    lines = []
    lines.append("| Planner | Path length | Total terrain cost | Risk exposure | Tracking error | Success |")
    lines.append("|---|---:|---:|---:|---:|---|")

    for row in rows:
        line = "| " + row["name"]
        line = line + " | " + str(row["length"])
        line = line + " | " + str(round(row["cost"], 1))
        line = line + " | " + str(row["risk"])
        line = line + " | " + str(round(row["tracking_error"], 2))
        line = line + " | " + row["success"] + " |"
        lines.append(line)

    table_text = "\n".join(lines)

    with open(file_name, "w", encoding="utf-8") as f:
        f.write(table_text)
        f.write("\n")

    return table_text


def main():
    output_folder = make_output_folder()

    grid, start, goal = make_grid_map()

    shortest_cost = make_cost_map(grid, "shortest")
    risk_aware_cost = make_cost_map(grid, "risk_aware")

    shortest_path = astar(grid, shortest_cost, start, goal)
    risk_path = astar(grid, risk_aware_cost, start, goal)

    if len(shortest_path) == 0:
        print("Shortest path was not found.")
        return

    if len(risk_path) == 0:
        print("Risk-aware path was not found.")
        return

    shortest_length, shortest_cost_value, shortest_risk, shortest_rough = calculate_path_metrics(
        shortest_path, grid, risk_aware_cost
    )

    risk_length, risk_cost_value, risk_risk, risk_rough = calculate_path_metrics(
        risk_path, grid, risk_aware_cost
    )

    shortest_waypoints = path_to_waypoints(shortest_path)
    risk_waypoints = path_to_waypoints(risk_path)

    shortest_tracking = simple_tracking(shortest_waypoints, grid)
    risk_tracking = simple_tracking(risk_waypoints, grid)

    shortest_success = "Yes"
    if not shortest_tracking[5]:
        shortest_success = "No"

    risk_success = "Yes"
    if not risk_tracking[5]:
        risk_success = "No"

    rows = []
    rows.append({
        "name": "Shortest path",
        "length": shortest_length,
        "cost": shortest_cost_value,
        "risk": shortest_risk,
        "tracking_error": shortest_tracking[2],
        "success": shortest_success,
    })
    rows.append({
        "name": "Risk-aware path",
        "length": risk_length,
        "cost": risk_cost_value,
        "risk": risk_risk,
        "tracking_error": risk_tracking[2],
        "success": risk_success,
    })

    shortest_file = os.path.join(output_folder, "planned_path_shortest.png")
    risk_file = os.path.join(output_folder, "planned_path_risk_aware.png")
    both_file = os.path.join(output_folder, "planner_comparison_same_map.png")
    tracking_file = os.path.join(output_folder, "planner_tracking_comparison.png")
    table_file = os.path.join(output_folder, "comparison_table.md")

    draw_one_path(grid, start, goal, shortest_path, "Shortest path planner", shortest_file)
    draw_one_path(grid, start, goal, risk_path, "Risk-aware planner", risk_file)
    draw_two_paths(grid, start, goal, shortest_path, risk_path, both_file)
    draw_tracking(
        grid,
        shortest_path,
        risk_path,
        (shortest_tracking[0], shortest_tracking[1]),
        (risk_tracking[0], risk_tracking[1]),
        tracking_file,
    )

    table_text = write_table(table_file, rows)

    print(table_text)
    print("")
    print("Saved output files in:")
    print(output_folder)


if __name__ == "__main__":
    main()