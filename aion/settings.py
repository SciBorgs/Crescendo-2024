from tests import theta_approx, v_approx
import numpy as np

"""
Returns an angle relative to the horizontal and launch speed for the shooter to reach while moving at 
a velocity (vx, vy) with position (x, y) aiming at the center of the target speaker.
"""

def get_shooter_state(x, y, vx, vy):
    initial_heading = np.arctan(y / x) if x!=0 else np.pi / 2
    target_v_stationary = v_approx(x, y)
    target_theta = theta_approx(x, y)
    delta_vx = target_v_stationary * np.cos(initial_heading) * np.cos(target_theta) - vx
    delta_vy = target_v_stationary * np.sin(initial_heading) * np.cos(target_theta) - vy
    set_heading = (np.arctan(delta_vy / delta_vx) if delta_vx != 0 else 0)
    launch_speed = np.sqrt( (delta_vx ** 2 + delta_vy ** 2 + (target_v_stationary*np.sin(target_theta)) ** 2) )
    return launch_speed, target_theta, set_heading

print(get_shooter_state(0, 4, -1, 0))



