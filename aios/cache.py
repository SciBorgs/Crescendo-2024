import json
from trajectory import Trajectory

interval = 2

points = []

for i in range(1, 6):
    for j in range(1, 6):
        points += [(i * interval, j * interval)]

results = {}

# this (among other things) doesn't actually work
def distToTarget(x, y):
    return (x ** 2 + y ** 2) ** 0.5

for x, y in points:
    # these numbers are almost entirely random
    t = Trajectory(distToTarget(x, y), 0.01, 0.3, 0, 0.8, 0.2)
    results[f"{x},{y}"] = t.getNewShooterState()


json_results = json.dumps(results, indent=4)

with open("cached_shooter_states.json", "w") as file:
    file.write(json_results)