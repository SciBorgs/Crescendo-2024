#!/usr/bin/env python3

"""FRC 2022 shooter trajectory optimization.

This program finds the optimal initial launch velocity and launch angle for the
2022 FRC game's target.
"""

from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import norm
import casadi as ca
import math
import matplotlib.pyplot as plt
import numpy as np

field_width = 8.2296  # 27 ft
field_length = 16.4592  # 54 ft

g = 9.806
max_launch_velocity = 10
speaker_max_height = 2.11  # m

min_launch_angle = 0
max_launch_angle = 1.1

# shooter = np.array([[field_length / 6.0], [field_width / 6.0], [0.635]])
# shooter_x = shooter[0, 0]
# shooter_y = shooter[1, 0]
# shooter_z = shooter[2, 0]

target = np.array([[0], [field_width / 2.0], [2]])
target_x = target[0, 0]
target_y = target[1, 0]
target_z = target[2, 0]
target_radius = 0.61


def lerp(a, b, t):
    return a + t * (b - a)


def hypot(a, b):
    return ca.sqrt(a**2 + b**2)


def f(x, alpha):
    # x' = x'
    # y' = y'
    # z' = z'
    # x" = −a_D(v_x)
    # y" = −a_D(v_y)
    # z" = −g − a_D(v_z)
    #
    # where a_D(v) = ½ρv² C_D A / m
    rho = 1.204  # kg/m³
    C_D0 = 0.08
    C_DA = 2.72
    C_D = C_D0 + C_DA * (alpha) ** 2  # paper uses degrees??
    # C_D = 0.05
    # A = math.pi * (0.356 / 2) ** 2
    diameter = 0.356
    width = 0.0508
    A_ring = math.pi * (diameter / 2) ** 2 - math.pi * ((diameter - width) / 2) ** 2
    A_rectangle = diameter * width
    A_D = A_rectangle * ca.cos(alpha) + A_ring * ca.sin(alpha)
    A_L = A_rectangle * ca.sin(alpha) + A_ring * ca.cos(alpha)

    m = 0.235301  # kg
    # accel due to drag
    a_D = lambda v: 0.5 * rho * v**2 * C_D * A_D / m
    # a_D = lambda v: 0

    # accel due to lift
    C_L = 0.15 + 1.4 * alpha
    a_L = lambda v: 0.5 * rho * v**2 * A_L * C_L / m
    # a_L = lambda v: 0

    v_x = x[3, 0]
    v_y = x[4, 0]
    v_z = x[5, 0]
    return ca.vertcat(v_x, v_y, v_z, -a_D(v_x), -a_D(v_y), -g + a_L(hypot(v_x, v_y)))


class Solver:
    shooter_z = 0.635

    def __init__(self) -> None:
        self._opti = ca.Opti()

        # Set up duration decision variables
        self.N = 20
        T = self._opti.variable()
        self._opti.subject_to(T >= 0)
        self._opti.set_initial(T, 1)
        dt = T / self.N

        #     [x position]
        #     [y position]
        #     [z position]
        # x = [x velocity]
        #     [y velocity]
        #     [z velocity]
        self.X = self._opti.variable(6, self.N)

        self.p_x = self.X[0, :]
        self.p_y = self.X[1, :]
        self.p_z = self.X[2, :]
        self.v_x = self.X[3, :]
        self.v_y = self.X[4, :]
        self.v_z = self.X[5, :]

        # Require initial launch velocity is below max
        # √{v_x² + v_y² + v_z²) <= vₘₐₓ
        # v_x² + v_y² + v_z² <= vₘₐₓ²
        self._opti.subject_to(
            self.v_x[0] ** 2 + self.v_y[0] ** 2 + self.v_z[0] ** 2
            <= max_launch_velocity**2
        )

        # Require final position is in center of target circle
        self._opti.subject_to(self.p_x[-1] == target_x)
        self._opti.subject_to(self.p_y[-1] == target_y)
        self._opti.subject_to(self.p_z[-1] == target_z)

        # Require the final velocity is down
        self._opti.subject_to(self.v_z[-1] > 0)

        # Assume the note never rotates (a bad assumption)
        pitch = ca.atan2(self.v_z[0], hypot(self.v_x[0], self.v_y[0]))
        # pitch = math.pi / 2.0 - ca.asin(ca.norm_1(self.X[:3, 0]) / ca.norm_1(self.X[:3, 0]))
        # Constrain starting angle
        self._opti.subject_to(pitch > min_launch_angle)
        self._opti.subject_to(pitch < max_launch_angle)

        # Dynamics constraints - RK4 integration
        for k in range(self.N - 1):
            h = dt
            x_k = self.X[:, k]
            x_k1 = self.X[:, k + 1]

            k1 = f(x_k, pitch)
            k2 = f(x_k + h / 2 * k1, pitch)
            k3 = f(x_k + h / 2 * k2, pitch)
            k4 = f(x_k + h * k3, pitch)
            self._opti.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

        # Avoid speaker physical constraints
        self._opti.subject_to(self.X[2] < speaker_max_height)

        # Minimize distance from goal over time
        J = 0
        for k in range(self.N):
            J += (target - self.X[:3, k]).T @ (target - self.X[:3, k])
        self._opti.minimize(J)

    def solve(self, x, y):
        # TODO add proper box constraints
        shooter = np.array([[x], [y], [self.shooter_z]])
        # Position initial guess is linear interpolation between start and end position
        for k in range(self.N):
            self._opti.set_initial(self.p_x[k], lerp(x, target_x, k / self.N))
            self._opti.set_initial(self.p_y[k], lerp(y, target_y, k / self.N))
            self._opti.set_initial(
                self.p_z[k], lerp(self.shooter_z, target_z, k / self.N)
            )

        # Velocity initial guess is max launch velocity toward goal
        uvec_shooter_to_target = target - shooter
        uvec_shooter_to_target /= norm(uvec_shooter_to_target)
        for k in range(self.N):
            self._opti.set_initial(
                self.v_x[k], max_launch_velocity * uvec_shooter_to_target[0, 0]
            )
            self._opti.set_initial(
                self.v_y[k], max_launch_velocity * uvec_shooter_to_target[1, 0]
            )
            self._opti.set_initial(
                self.v_z[k], max_launch_velocity * uvec_shooter_to_target[2, 0]
            )

        # Shooter initial position
        self._opti.subject_to(self.X[:3, 0] == shooter)

        self._opti.solver("ipopt")
        sol = self._opti.solve()
        return sol

    def optimal_settings(self, x, y):

        # TODO definitely wrong, return None and ignore instead of 0s
        try:
            sol = self.solve(x, y)
            v = sol.value(self.X[3:, 0])

            # From Tyler
            # The launch angle is the angle between the initial velocity vector and the x-y
            # plane. First, we'll find the angle between the z-axis and the initial velocity
            # vector.
            #
            # sinθ = |a x b| / (|a| |b|)
            #
            # Let v be the initial velocity vector and p be a unit vector along the z-axis.
            #
            # sinθ = |v x p| / (|v| |p|)
            # sinθ = |v x [0, 0, 1]| / |v|
            # sinθ = |[v_y, -v_x, 0]|/ |v|
            # sinθ = √(v_x² + v_y²) / |v|
            #
            # The square root part is just the norm of the first two components of v.
            #
            # sinθ = |v[:2]| / |v|
            #
            # θ = asin(|v[:2]| / |v|)
            #
            # The angle between the initial velocity vector and the X-Y plane is 90° − θ.
            launch_angle = math.pi / 2.0 - math.asin(norm(v[:2]) / norm(v))
            launch_velocity = norm(v)

            print(f"Launch velocity = {round(launch_velocity, 3)} m/s")
            print(f"Launch angle = {round(launch_angle * 180.0 / math.pi, 3)}°")
            return (launch_velocity, launch_angle)
        except:
            return (0, 0)

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

        # Target
        ax.plot(
            target_x,
            target_y,
            target_z,
            color="black",
            marker="x",
        )
        xs = []
        ys = []
        zs = []
        for angle in np.arange(0.0, 2.0 * math.pi, 0.1):
            xs.append(target_x + target_radius * math.cos(angle))
            ys.append(target_y + target_radius * math.sin(angle))
            zs.append(target_z)
        ax.plot(xs, ys, zs, color="black")

        # Trajectory
        trajectory_x = sol.value(self.p_x)
        trajectory_y = sol.value(self.p_y)
        trajectory_z = sol.value(self.p_z)
        ax.plot(trajectory_x, trajectory_y, trajectory_z, color="orange")

        print(
            "succeeded !!! ",
            math.atan2(
                sol.value(self.v_z)[0],
                math.sqrt(sol.value(self.v_x)[0] ** 2 + sol.value(self.v_y)[0] ** 2),
            )
            * 180
            / math.pi,
        )

        ax.set_box_aspect((field_length, field_width, np.max(trajectory_z)))

        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.set_zlabel("Z position (m)")

        plt.show()


if __name__ == "__main__":
    s = Solver()

    s.visualize(field_length / 3, field_width / 3)

    # print(s._opti.debug.value)
