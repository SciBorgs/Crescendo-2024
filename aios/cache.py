import json
from trajectory import Trajectory
from math import asin

interval = 0.5

points : list[tuple[float, float]] = []

for i in range(-15, 15):
    for j in range(-15, -1):
        points += [(i * interval, j * interval)]

results = {}

def angle_to_origin(x, y):
    dist = (x**2 + y**2)**0.5
    return asin(- x / dist)

failures = []

for x, y in points:
    # these numbers are almost entirely random
    if Trajectory.canShootWhileStationary(x, y):
        t = Trajectory(x, y, 0, 0, angle_to_origin(x, y), 0, 1.104793, 2, 1, 3)
        try:
            results[f"{x},{y}"] = t.getNewShooterState()
        except:
            failures += [(x, y)]


print("\nfailures2: " + str(failures) + "\nyesses: ")

json_results = json.dumps(results, indent=4)

with open("src/main/deploy/shooter_trajectories_cache.json", "w") as file:
    file.write(json_results)