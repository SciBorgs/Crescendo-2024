from frispy import disc
import numpy as np

note = disc.Disc()

#change attributes to match note
"""
note.area = 
note.I_xx = 
note.I_zz = 
note.mass = 
"""

max_launch_speed = 7 #m/s
min_angle = 0 #rad
max_angle = 1.1 #rad
distance_to_front_target = 1 #m
distance_to_back_target = distance_to_front_target + 0.46 #m
max_height = 2.11 #m
min_height = 2.02 #m
min_launch_speed = np.sqrt(2 * 9.8 * max_height) #if "right below target"
launch_speed = max_launch_speed #initial guess

while launch_speed > min_launch_speed:
    angle = max_angle
    while angle > min_angle:
        note.vz = launch_speed * np.sin(angle) #m/s
        note.vx = launch_speed * np.cos(angle) #m/s
        time_to_front_target = distance_to_front_target / ( (launch_speed) * np.cos(angle) )
        trajectory, results = note.compute_trajectory(flight_time = time_to_front_target)
        height_at_front_target = trajectory['z'][-1]
        
        time_to_back_target = distance_to_back_target / ( (launch_speed) * np.cos(angle) )
        trajectory, results = note.compute_trajectory(flight_time = time_to_back_target)
        height_at_back_target = trajectory['vz'][-1]
        print(launch_speed, angle, height_at_front_target, height_at_back_target)
        if height_at_front_target < max_height and height_at_back_target > min_height:
            break
        angle -= 0.01
    launch_speed -= 0.05

    
    
