import numpy as np

import differential_flatness as df


def publish_hovering(hovering_point):
    """
    This function is to generate hovering state at final point.
    As you see, except position and psi, all state must be zero to hover drone.
    It returns trajectory which is computed by differential flatness theory.
    """

    pos = hovering_point
    x = pos[0]
    y = pos[1]
    z = pos[2]
    psi = pos[3]

    pos = np.array([x, y, z])
    vel = np.array([0, 0, 0])
    acc = np.array([0, 0, 0])
    jerk = np.array([0, 0, 0])
    snap = np.array([0, 0, 0])
    yaw = psi
    yaw_dot = 0
    yaw_ddot = 0
    flat_output = [pos, vel, acc, jerk, snap, yaw, yaw_dot, yaw_ddot]
    ref_trajectory = df.compute_ref(flat_output)

    return ref_trajectory
