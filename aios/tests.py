from trajectory import Trajectory

t1 = Trajectory(5, 13, 0.25, 0, 0.5, 0.5)
solutions1 = t1.getNewShooterState()

print(solutions1['angle'])
print(solutions1['speed'])

t2 = Trajectory(4, 13, 0.25, 0, 0.5, 0.5)
solutions2 = t2.getNewShooterState()

print(solutions2['angle'])
print(solutions2['speed'])

t3 = Trajectory(5, 13, 0.25, 0, 0.5, 0.5 )
solutions3 = t3.getNewShooterState()

print(solutions3['angle'])
print(solutions3['speed'])

t4 = Trajectory(6, 25, 0.5, 0, 0.5, 0.5)
solutions4 = t4.getNewShooterState()
print(solutions4['angle'])
print(solutions4['speed'])

t5 = Trajectory(15, 3, 0.5, 0, 0.5, 0.5)
solutions5 = t5.getNewShooterState()
print(solutions5['angle'])
print(solutions5['speed'])

