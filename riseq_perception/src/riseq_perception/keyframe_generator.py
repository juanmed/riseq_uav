import numpy as np
import tf


def gate_keyframe(current_pose, gate_location, gate_count):
    """
    Function to get keyframe, only for simulator
    In real experiment, other approach of getting keyframe is needed.
    """
    # if pose is written as quaternion form, it needs to be transformed to euler form.
    # if current_pose == [ x y z x y z w ]   3 position 4 quaternion  -> 3 position and 1 yaw
    if len(current_pose) == 7:
        current_pose_orientation = tf.transformations.euler_from_quaternion(
            [current_pose[3], current_pose[4], current_pose[5], current_pose[6]], axes='sxyz')
        current_pose = np.array([current_pose[0], current_pose[1], current_pose[2], current_pose_orientation[2]])

    keyframe = np.zeros((gate_count+1, 4))
    keyframe[0] = current_pose

    for i in range(0, gate_count):
        keyframe[i+1] = gate_point(gate_location[i])
    keyframe = compensate_direction(keyframe, gate_count)
    return keyframe


def gate_point(gate):
    """
    Function to get way point of each gate
    return position and psi
    """
    # calculate center point of gate
    gate_x = np.sum(gate[:, 0]) / 4.0
    gate_y = np.sum(gate[:, 1]) / 4.0
    gate_z = np.sum(gate[:, 2]) / 4.0

    # cross product for getting gate direction
    p1 = gate[0, :]
    p2 = gate[1, :]
    p3 = gate[2, :]
    v1 = p3 - p1
    v2 = p2 - p1
    cp = np.cross(v1, v2)
    gate_psi = np.arctan2(cp[1], cp[0])

    gate_pose = np.array([gate_x, gate_y, gate_z, gate_psi])
    return gate_pose


def compensate_direction(keyframe, gate_count):
    """
    Function to compensate direction of gate point.
    Sometimes there are reverse direction.
    """
    for i in range(gate_count):
        before_keyframe = keyframe[i]
        after_keyframe = keyframe[i+1]
        x_vector = after_keyframe[0] - before_keyframe[0]
        y_vector = after_keyframe[1] - before_keyframe[1]
        z_vector = after_keyframe[2] - before_keyframe[2]
        vector = np.array([x_vector, y_vector, z_vector])
        direction = ([np.cos(after_keyframe[3]), np.sin(after_keyframe[3]), 0])
        dotproduct = np.dot(vector, direction)
        cosangle = dotproduct / (np.linalg.norm(vector) * np.linalg.norm(direction))
        if cosangle < 0:
            if after_keyframe[3] < 0:
                after_keyframe[3] = after_keyframe[3] + np.pi
            else:
                after_keyframe[3] = after_keyframe[3] - np.pi
        keyframe[i+1][3] = after_keyframe[3]

    return keyframe
