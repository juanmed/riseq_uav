import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_traj3D(solution, order, m, keyframe):
    """
    Plot trajectory from 3 dimension [ x y z ] solution
    """
    n = 3
    x_trajec = []
    y_trajec = []
    z_trajec = []

    for i in range(0, m):
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

    # plot x y z
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('xyz')
    ax.plot(x_trajec, y_trajec, z_trajec, 'r')
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-60, 60)
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    for i in range(0, len(keyframe)):
        ax.text(keyframe[i][0], keyframe[i][1], keyframe[i][2], i, color='red')
    plt.show()


def plot_traj(solution, order, m, keyframe, n=4):
    """
    Plot trajectory from 4 dimension [ x y z psi ] solution
    """

    x_trajec = []
    y_trajec = []
    z_trajec = []
    psi_trajec = []

    for i in range(0, m):
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
        if n == 4:
            psi_trajec = np.append(psi_trajec, np.polyval(
                solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)],
                np.linspace(0, 1, 50)))

    # plot x y z
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('xyz')
    ax.plot(x_trajec, y_trajec, z_trajec, 'r')
    ax.set_xlim(-60, 60)
    ax.set_ylim(-60, 60)
    ax.set_zlim(-60, 60)
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


def plot_vel(solution, order, waypoint, keyframe, time_scaling):
    x_vel = []
    y_vel = []
    z_vel = []
    psi_vel = []
    n = 4

    for i in range(0, waypoint-1):
        # we can use np.arrange instead of np.linspace
        x_vel = np.append(x_vel, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)], 1),
            np.linspace(0, 1/time_scaling[i], 50)))
        y_vel = np.append(y_vel, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)], 1),
            np.linspace(0, 1/time_scaling[i], 50)))
        z_vel = np.append(z_vel, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)], 1),
            np.linspace(0, 1/time_scaling[i], 50)))
        psi_vel = np.append(psi_vel, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)], 1),
            np.linspace(0, 1/time_scaling[i], 50)))

        # plot x y z
        fig = plt.figure(2)
        axx = fig.add_subplot(211)
        axy = fig.add_subplot(212)
        axz = fig.add_subplot(213)
        axpsi = fig.add_subplot(214)
        axx.set_title('x_vel')
        axy.set_title('y_vel')
        axz.set_title('z_vel')
        axpsi.set_title('psi_vel')
        axx.set_xlim(-30, 30)
        axy.set_ylim(-30, 30)
        axz.set_zlim(-30, 30)
        axpsi.set_zlim(-3.14, 3.14)

        for i in range(0, len(keyframe)):
            axx.text(keyframe[i][0], i, color='red')
            axy.text(keyframe[i][1], i, color='red')
            axz.text(keyframe[i][2], i, color='red')
            axpsi.text(keyframe[i][3], i, color='red')

        plt.show()


def plot_acc(solution, order, waypoint, keyframe, time_scaling):
    x_acc = []
    y_acc = []
    z_acc = []
    psi_acc = []
    n = 4

    for i in range(0, waypoint - 1):
        # we can use np.arrange instead of np.linspace
        x_acc = np.append(x_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        y_acc = np.append(y_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        z_acc = np.append(z_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        psi_acc = np.append(psi_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))

        # plot x y z
        fig = plt.figure(2)
        axx = fig.add_subplot(211)
        axy = fig.add_subplot(212)
        axz = fig.add_subplot(213)
        axpsi = fig.add_subplot(214)
        axx.set_title('x_acc')
        axy.set_title('y_acc')
        axz.set_title('z_acc')
        axpsi.set_title('psi_acc')
        axx.set_xlim(-30, 30)
        axy.set_ylim(-30, 30)
        axz.set_zlim(-30, 30)
        axpsi.set_zlim(-3.14, 3.14)

        for i in range(0, len(keyframe)):
            axx.text(keyframe[i][0], i, color='red')
            axy.text(keyframe[i][1], i, color='red')
            axz.text(keyframe[i][2], i, color='red')
            axpsi.text(keyframe[i][3], i, color='red')

        plt.show()