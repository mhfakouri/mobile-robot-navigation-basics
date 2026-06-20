import os
import math
import numpy as np
import matplotlib.pyplot as plt

this_file = os.path.abspath(__file__)
this_folder = os.path.dirname(this_file)

# going up from:
# projects/04_vehicle_tracking/
# to:
# mobile_robot_navigation_basics/
main_folder = os.path.dirname(os.path.dirname(this_folder))

output_folder = os.path.join(main_folder, "outputs")

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

print("output folder is:", output_folder)

# This is just a simple path.
# Later, this can be replaced by the A star path.
waypoints = np.array([
    [0, 0],
    [3, 1],
    [6, 1],
    [8, 4],
    [10, 7],
    [13, 8]
], dtype=float)

# robot initial state
x = 0.0
y = 0.0
theta = 0.0

dt = 0.1
v = 0.8

# this gain turns robot toward the next waypoint
k = 2.0

target_index = 1

x_list = []
y_list = []

i = 0

while i < 600:

    target_x = waypoints[target_index, 0]
    target_y = waypoints[target_index, 1]

    dx = target_x - x
    dy = target_y - y

    distance = math.sqrt(dx*dx + dy*dy)

    # if robot is close to current target, go to next target
    if distance < 0.3:
        if target_index < len(waypoints) - 1:
            target_index = target_index + 1
        else:
            break

    target_x = waypoints[target_index, 0]
    target_y = waypoints[target_index, 1]

    dx = target_x - x
    dy = target_y - y

    desired_theta = math.atan2(dy, dx)

    error_theta = desired_theta - theta

    # keep angle error between -pi and pi
    error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

    omega = k * error_theta

    # simple unicycle model
    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt
    theta = theta + omega * dt

    x_list.append(x)
    y_list.append(y)

    i = i + 1

print("simulation steps:", len(x_list))
print("final x:", x)
print("final y:", y)
print("last target index:", target_index)

plt.figure()

plt.plot(waypoints[:, 0], waypoints[:, 1], marker="o", label="waypoints")
plt.plot(x_list, y_list, label="robot path")

plt.title("Practice 04 - simple vehicle tracking")
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.legend()

save_name = os.path.join(output_folder, "practice_04_vehicle_tracking.png")
plt.savefig(save_name)
print("saved:", save_name)
plt.show()

print("saved: outputs/practice_04_vehicle_tracking.png")