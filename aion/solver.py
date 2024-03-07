#!/usr/bin/env python3

"""
SciBorgs, FRC 1155.
This program is free to use under the MIT license.
Initially based on a program by Tyler Veness to optimize cargo trajectories in Rapid React (2022).
"""

from numpy.linalg import norm
import casadi as ca
import math
import matplotlib.pyplot as plt
import numpy as np

field_width = 8.2296  # 27 ft
field_length = 16.4592  # 54 ft

g = 9.806
max_launch_velocity = 20

min_launch_angle = 0
max_launch_angle = 1.1

speaker_low_edge = 2.002
speaker_top_edge = 2.124
inclined_top_angle = 14 * (np.pi / 180)  # rad
delta_y = 1.051  # length of speaker (parallel to wall)
delta_x = 0.451  # x distance from wall to start of speaker
delta_z = 0.516  # height of window to score into
bar_height = delta_z - (2 * delta_x * np.tan(inclined_top_angle))  # m
starting_slanted_height = speaker_top_edge + bar_height  # m

note_diameter = 0.356
note_width = 0.0508

target = np.array([[0], [field_width / 2.0], [2]])
target_x = target[0, 0]
target_y = target[1, 0]
target_z = target[2, 0]
target_radius = 0.61

# height of shooter
shooter_z = 0.635
shooter_offset_x = 0.2315972
shooter_offset_z = 0.1490472


def lerp(a, b, t):
    return a + t * (b - a)


def hypot(a, b):
    return ca.sqrt(a**2 + b**2)


y_center = field_width / 2


def f(x, alpha):
    """
    A bad approximation of the dynamics of a note, roughly based on https://web.mit.edu/womens-ult/www/smite/frisbee_physics.pdf
    """
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

    # A = math.pi**2 * note_width * note_diameter / 2
    A_ring = (
        math.pi * (note_diameter / 2) ** 2
        - math.pi * ((note_diameter - note_width) / 2) ** 2
    )
    A_rectangle = note_diameter * note_width
    A_D = A_rectangle * ca.cos(alpha) + A_ring * ca.sin(alpha)
    A_L = A_rectangle * ca.sin(alpha) + A_ring * ca.cos(alpha)

    # TODO add torque and angle changing?

    m = 0.235301  # kg
    # accel due to drag
    a_D = lambda v: 0.5 * rho * v**2 * C_D * A_D / m

    # accel due to lift
    C_L0 = 0.15
    C_LA = 1.4
    C_L = (C_L0 + C_LA * alpha) / 2
    a_L = lambda v: 0.5 * rho * v**2 * A_L * C_L / m

    v_x = x[3, 0]
    v_y = x[4, 0]
    v_z = x[5, 0]
    return ca.vertcat(v_x, v_y, v_z, -a_D(v_x), -a_D(v_y), -g + a_L(hypot(v_x, v_y)))


def danger_zone(p1: tuple[float], p2: tuple[float]):
    return through_front(p1, p2) + through_side(p1, p2)


def through_front(p1: tuple[float], p2: tuple[float]):
    def in_front(y, z):
        return (
            (y_center - delta_y / 2 < y)
            * (y < y_center + delta_y / 2)
            * (speaker_top_edge < z)
        )

    # def interp():
    line = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])

    dydx = line[1] / line[0]
    dzdx = line[2] / line[0]

    def y(x):
        return dydx * x + p1[1] - dydx * p1[0]

    def z(x):
        return dzdx * x + p1[2] - dzdx * p1[0]

    on_plane = (delta_x, y(delta_x), z(delta_x))

    return in_front(on_plane[1], on_plane[2]) * (p1[0] > delta_x) * (delta_x > p2[0])


def through_side(p1: tuple[float], p2: tuple[float]):
    m0 = (speaker_top_edge - speaker_low_edge) / delta_x
    b0 = speaker_top_edge - m0 * delta_x

    def in_side(x, z):
        return (0 < x) * (x < delta_x) * (m0 * x + b0 < z)

    def interp(y):
        line = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])

        dxdy = line[0] / line[1]
        dzdy = line[2] / line[1]

        def x(y):
            return dxdy * y + p1[0] - dxdy * p1[1]  # m x + b

        def z(y):
            return dzdy * y + p1[2] - dzdy * p1[1]  # m x + b

        return (x(y), y, z(y))

    left_y = y_center - delta_y / 2
    right_y = y_center + delta_y / 2
    y1 = p1[1]
    y2 = p2[1]
    left = interp(left_y)
    right = interp(right_y)
    in_left = in_side(left[0], left[2]) * (y1 < left_y) * (left_y < y2)
    in_right = in_side(right[0], right[2]) * (y1 > right_y) * (right_y > y2)
    return in_left + in_right


class Solver:
    """
    Solves an optimization problem to find the optimal pitch and velocity of a static shot from a specific coordinate into the subwoofer
    """

    def __init__(self) -> None:
        self._opti = ca.Opti()
        # self._opti.solver("ipopt", {"print_level": 0})
        self._opti.solver("ipopt")

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

        # def point(i):
        #     return (self.p_x[i], self.p_y[i], self.p_z[i])

        # for i in range(11, self.N - 5):
        #     self._opti.subject_to(
        #         danger_zone(point(i), point(i + 1)) == 0
        #     )  # checking that it's false

        # Require initial launch velocity is below max
        # √{v_x² + v_y² + v_z²) <= vₘₐₓ
        # v_x² + v_y² + v_z² <= vₘₐₓ²
        self._opti.subject_to(
            self.v_x[0] ** 2 + self.v_y[0] ** 2 + self.v_z[0] ** 2
            <= max_launch_velocity**2
        )

        # Require final position is in center of the speaker
        self._opti.subject_to(self.p_x[-1] == 0.2167)
        self._opti.subject_to(self.p_y[-1] == 5.549)
        self._opti.subject_to(self.p_z[-1] == 2.12)

        # Require final position is within speaker wall bounds
        # self._opti.subject_to(self.p_x[-1] < 0)  # note will be at wall
        # self._opti.subject_to(
        #     (field_width - note_diameter) / 2 - delta_y / 2 < self.p_y[-1]
        # )
        # self._opti.subject_to(
        #     (field_width - note_diameter) / 2 + delta_y / 2 > self.p_y[-1]
        # )
        # self._opti.subject_to(
        #     speaker_low_edge - (delta_z - note_width) / 2 < self.p_z[-1]
        # )
        # self._opti.subject_to(
        #     speaker_low_edge + (delta_z - note_width) / 2 > self.p_z[-1]
        # )

        # Require all velocities are going towards the wall
        self._opti.subject_to(self.v_x < 0)

        # Calculate initial pitch
        self.pitch = ca.atan2(self.v_z[0], hypot(self.v_x[0], self.v_y[0]))
        # pitch = math.pi / 2.0 - ca.asin(ca.norm_1(self.X[:3, 0]) / ca.norm_1(self.X[:3, 0]))
        # Constrain starting angle
        self._opti.subject_to(self.pitch > min_launch_angle)
        self._opti.subject_to(self.pitch < max_launch_angle)

        # Dynamics constraints - RK4 integration
        for k in range(self.N - 1):
            h = dt
            x_k = self.X[:, k]
            x_k1 = self.X[:, k + 1]

            k1 = f(x_k, self.pitch)
            k2 = f(x_k + h / 2 * k1, self.pitch)
            k3 = f(x_k + h / 2 * k2, self.pitch)
            k4 = f(x_k + h * k3, self.pitch)
            self._opti.subject_to(x_k1 == x_k + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))

        # Minimize distance from goal over time
        J = 0
        for k in range(self.N):
            J += (target - self.X[:3, k]).T @ (target - self.X[:3, k])
        self._opti.minimize(J)

    def solve(self, x, y):
        # x and y are pivot positions (i am lazy)
        # we offset pivot positons
        shooter = np.array([[x], [y], [shooter_z ]]) # + shooter_offset_z * ca.cos(self.pitch) + shooter_offset_x * ca.sin(self.pitch)
        # Position initial guess is linear interpolation between start and end position
        for k in range(self.N):
            self._opti.set_initial(self.p_x[k], lerp(x, target_x, k / self.N))
            self._opti.set_initial(self.p_y[k], lerp(y, target_y, k / self.N))
            self._opti.set_initial(self.p_z[k], lerp(shooter_z, target_z, k / self.N))

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

        sol = self._opti.solve()
        return sol

    def optimal_settings(self, x, y):
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
            print("couldn't find trajectory")
            return None

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
        target_xs = []
        target_ys = []
        target_zs = []

        front_hood_xs = []
        front_hood_ys = []
        front_hood_zs = []

        for y in np.arange(y_center - delta_y / 2, y_center + delta_y / 2, 0.005):
            for z in np.arange(speaker_low_edge, speaker_low_edge + delta_z, 0.005):
                target_xs += [0]
                target_ys += [y]
                target_zs += [z]
            for z in np.arange(speaker_top_edge, speaker_low_edge + delta_z, 0.005):
                front_hood_xs += [delta_x]
                front_hood_ys += [y]
                front_hood_zs += [z]

        ax.plot(front_hood_xs, front_hood_ys, front_hood_zs, color="red")
        ax.plot(target_xs, target_ys, target_zs, color="blue")

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
    s = Solver()

    s.visualize(4, 3 * field_width / 4)

    # print(s._opti.debug.value)
