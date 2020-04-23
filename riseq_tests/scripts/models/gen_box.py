import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D





def main():
    height = 0.13  # z axis
    width = 0.21 # y axis
    depth = 0.21 # x axis
    r = 0
    g = 255
    b = 0
    n = 100

    points = list()
    s = [height/2., -height/2., width/2., -width/2., depth/2., -depth/2.]

    for i,s in enumerate(s):
        if i == 0 or i == 1:
            x = -width/2 + np.random.random_sample(n)*width
            y = -depth/2 + np.random.random_sample(n)*depth
            z = np.ones_like(x)*s
            for a,b,c in zip(x,y,z):
                points.append([a,b,c,r,g,b])

        elif i == 2 or i == 3:
            z = -height/2 + np.random.random_sample(n)*height
            x = -depth/2 + np.random.random_sample(n)*depth
            y = np.ones_like(x)*s
            for a,b,c in zip(x,y,z):
                points.append([a,b,c,r,g,b])        
        elif i == 4 or i == 5:
            z = -height/2 + np.random.random_sample(n)*height
            y = -width/2 + np.random.random_sample(n)*width
            x = np.ones_like(x)*s
            for a,b,c in zip(x,y,z):
                points.append([a,b,c,r,g,b])  
        else:
            print("Invalid surface")

    points = np.array(points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:,0], points[:,1], points[:,2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()



if __name__ == '__main__':
    main()


