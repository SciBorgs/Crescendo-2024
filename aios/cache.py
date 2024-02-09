import json
from trajectory import Trajectory
from math import asin

interval = 0.5

points : list[tuple[float, float]] = []

for i in range(-15, 15):
    for j in range(-15, -1):
        points += [(i * interval, j * interval)]

results = {}

# TODO test this!!!
def angle_to_origin(x, y):
    dist = (x**2 + y**2)**0.5
    return asin(- x / dist)

failures = []

for x, y in points:
    if Trajectory.canShootWhileStationary(x, y):
        t = Trajectory(x, y, 0, 0, angle_to_origin(x, y), 0, 1.104793, 0.33, 0.33, 0.33)
        try:
            results[f"{x},{y}"] = t.getNewShooterState()
        except:
            failures += [(x, y)]


print("\nfailures2: " + str(failures))

json_results = json.dumps(results, indent=4)

with open("src/main/deploy/shooter_trajectories_cache.json", "w") as file:
    file.write(json_results)