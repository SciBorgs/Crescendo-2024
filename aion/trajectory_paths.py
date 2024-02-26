import numpy as np

class Trajectory:
    density_of_air = 1.204 #kg/m^3
    dynamic_viscosity = 1.825 * (10 ** -5) #kg/(m*s)
    note_small_radius = 0.05 / 2 #m
    note_large_radius = 0.36 / 2 #m
    note_thickness = 0.05 #m

    volume = 2 * np.pi * note_large_radius * (note_small_radius ** 2)
    surface_area = 4 * np.pi * note_small_radius * note_large_radius
    characteristic_length = volume / surface_area
    note_frontal_area = np.pi * (note_large_radius) ** 2
    aspect_ratio = (2*(note_large_radius - note_small_radius)) / (2 * note_small_radius)

    def __init__(self, v, theta):
        self.v = v #initial velocity
        self.theta = theta
        self.vx = np.cos(theta) #initial horizontal velocity
        self.vy = np.sin(theta) #initial vertical velocity
    
    def net_force(self):
        Re = 2 * self.density_of_air * self.v * self.note_large_radius / self.dynamic_viscosity
        drag_coefficent = 0.207485 * ( self.aspect_ratio ** (0.626308) ) * np.cos(2 * self.theta) + (5.781323) * Re ** (-0.468003)
        drag_force = 0.5 * drag_coefficent * self.density_of_air * (self.v ** 2) * self.note_frontal_area * np.sin(self.theta)
        lift_coefficent = 0.987067 * self.aspect_ratio ** (-0.115132) * Re ** (-0.298905) * np.sin(2*self.theta)
        lift_force = 0.5 * lift_coefficent * self.density_of_air * (self.v ** 2) * self.note_frontal_area * np.sin(self.theta)
        return lift_force, drag_force
    

t1 = Trajectory(3, np.pi/3)
print(t1.net_force())


"""   
def visualize(self, x, y):

        # Initial velocity vector
        sol = self.solve(x, y)

        v = sol.value(self.X[3:, 0])

        launch_velocity = norm(v)
        print(f"Launch velocity = {round(launch_velocity, 3)} m/s")

        launch_angle = math.pi / 2.0 - math.asin(norm(v[:2]) / norm(v))
        print(f"Launch angle = {round(launch_angle * 180.0 / math.pi, 3)}°")

        fig = plt.figure()
        ax = plt.axes(projection="3d")

        def plot_wireframe(ax, f, x_range, y_range, color):
            x, y = np.mgrid[
                x_range[0] : x_range[1] : 25j, y_range[0] : y_range[1] : 25j
            ]

            # Need an (N, 2) array of (x, y) pairs.
            xy = np.column_stack([x.flat, y.flat])

            z = np.zeros(xy.shape[0])
            for i, pair in enumerate(xy):
                z[i] = f(pair[0], pair[1])
            z = z.reshape(x.shape)

            ax.plot_wireframe(x, y, z, color=color)

        # Ground
        plot_wireframe(
            ax, lambda x, y: 0.0, [0, field_length], [0, field_width], "grey"
        )

        # Trajectory
        trajectory_x = sol.value(self.p_x)
        trajectory_y = sol.value(self.p_y)
        trajectory_z = sol.value(self.p_z)
        ax.plot(trajectory_x, trajectory_y, trajectory_z, color="orange")

        ax.set_box_aspect((field_length, field_width, np.max(trajectory_z)))

        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.set_zlabel("Z position (m)")

        plt.show()


if __name__ == "__main__":
    visualize()

    # print(s._opti.debug.value)
"""