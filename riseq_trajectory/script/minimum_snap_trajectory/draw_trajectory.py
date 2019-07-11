import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw_in_plot(solution, order, waypoint, keyframe):
    x_trajec = []
    y_trajec = []
    z_trajec = []
    psi_trajec = []
    n = 4

    for i in range(0, waypoint-1):
        # we can use np.arrange instead of np.linspace
        x_trajec = np.append(x_trajec, np.polyval(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)],
            np.linspace(0, 1, 50)))
        y_trajec = np.append(y_trajec, np.polyval(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)],
            np.linspace(0, 1, 50)))
        z_trajec = np.append(z_trajec, np.polyval(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)],
            np.linspace(0, 1, 50)))
        psi_trajec = np.append(psi_trajec, np.polyval(
            solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)],
            np.linspace(0, 1, 50)))

    # plot x y z
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('xyz')
    ax.plot(x_trajec, y_trajec, z_trajec, 'r')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-20, 20)

    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    for i in range(0, len(keyframe)):
        ax.text(keyframe[i][0], keyframe[i][1], keyframe[i][2], i, color='red')
    '''
    # plot Yaw
    ax2 = fig.add_subplot(212)
    ax2.set_title('psi')
    ax2.set_ylim(-3.14, 3.14)
    ax2.plot(psi_trajec, 'b')
    '''
    plt.show()