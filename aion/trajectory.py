import casadi
import numpy as np

#Define needed constants
g = 9.8 #m/s/s
speaker_width = 0.46 #m
max_speaker_height = 2.11 #m
min_speaker_height = 1.98 #m
note_thickness = 0.06 #m
note_outer_radius = 0.36 #m
max_launch_speed = 13 #m/s
speaker_length = 1.05 #m

starting_shooter_height = 0 #m



class Trajectory:
    """
    Represents a trajectory to get the note into the speaker as a function of the robot position.
    It is assumed that the robot aims its shooter at the front center of the speaker (defined as the origin).
    
    The loss function is defined as the square of the changes in launch speed and angle, 
    from an assumed starting state of half the maximum launch speed and launch angle (45 degrees).
    Since optimization will not be performed in realtime, this provides the least amount of 'bias'
    towards any particular starting state when the robot reaches the given position (x,y).
    """


    def __init__(self, posX, posY):
        self.posX = posX
        self.posY = posY

    def get_optimal_settings(self):
        """
        Returns an optimal launch speed and launch angle with respect to the constraints described above.
        This is done using casadi optimization, minimizing the defined loss function with two constraints:
        1 - The note is below the maximum first entry height of the speaker when it reaches the corresponding horizontal position.
        2 - The note is above the minimum second entry height of the speaker when it reaches the corresponding horizontal position.
        """

        optimizer = casadi.Opti()
        delta_theta = optimizer.variable()
        delta_v = optimizer.variable()
        
        initial_v = 0.0001
        initial_theta = 0.0001
        heading = abs(np.arctan(self.posY / self.posX)) if self.posX != 0 else np.pi / 2
        
        t_1 = (self.posY) / ( (initial_v + delta_v) * casadi.cos(initial_theta + delta_theta) * np.sin(heading) )
        t_2 = (self.posY + speaker_width) / ( (initial_v + delta_v) * casadi.cos(initial_theta + delta_theta) * np.sin(heading) )

        loss_function = (delta_theta) ** 2 + (delta_v) ** 2
        optimizer.minimize(loss_function)

        optimizer.subject_to(
            max_speaker_height - (note_thickness / 2) > 
            ( (initial_v + delta_v) * casadi.sin(initial_theta + delta_theta) * t_1 ) - ( (1/2) * g * t_1 ** 2)
        )

        optimizer.subject_to(
            min_speaker_height + (note_thickness / 2) <
            ( (initial_v + delta_v) * casadi.sin(initial_theta + delta_theta) * t_2 ) - ( (1/2) * g * t_2 ** 2)
        )

        optimizer.subject_to(
            0 < delta_v + initial_v
        )
        optimizer.subject_to(
            delta_v + initial_v <= max_launch_speed
        )

        optimizer.subject_to(
            0 < delta_theta + initial_theta
        )

        optimizer.subject_to(
            delta_theta + initial_theta < np.pi / 2
        )

        optimizer.solver('ipopt')
        try:
            solutions = optimizer.solve()
        except:
            return [0, 0]
        return [(solutions.value(delta_v) + initial_v), (solutions.value(delta_theta) + initial_theta)]



    
        