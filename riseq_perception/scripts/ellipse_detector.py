"""
author:  author's name
version: version of this script
brief: a description of the functionality this script implements

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import cv2
import rospy
import argparse
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.widgets import Slider


def build_arg_parser():
    """
    Build command line argument parser
    Return:
        Dictionary containing arguments and their values
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required = True, help = "Path to the image")
    ap = ap.parse_args()
    return ap   

class Ellipse():
    def __init__(self,a=1,b=1,cx=0,cy=0,theta=0):
        """
        Create an ellipse object
        Args:
            a --  major radius
            b -- minor radius
            cx -- x coordinate of ellipse center
            cy -- y coordinate of ellipse center
            theta -- angle by which the ellipse is rotated
        """
        self.a = a
        self.b = b
        self.cx = cx
        self.cy = cy
        self.theta = theta

    def get_ellipse_point(self,alpha):
        """
        Get x,y values corresponding to angle alpha
        Args:
            theta -- angle for which the x,y coordinated will be computed
        Returns:
            x,y values of the point corresponding to the angle alpha
        See: https://math.stackexchange.com/questions/2645689/what-is-the-parametric-equation-of-a-rotated-ellipse-given-the-angle-of-rotatio
        """
        x = self.a*np.cos(alpha)*np.cos(self.theta) - self.b*np.sin(alpha)*np.sin(self.theta) + self.cx
        y = self.a*np.cos(alpha)*np.sin(self.theta) + self.b*np.sin(alpha)*np.cos(self.theta) + self.cy
        return x,y

class EllipseDetector():

    def __init__(self, mode = "test"):

        self.max_size = 800
        self.img = 0

        self.centroid_threshold = 10
        # Canny parameters
        self.lt = 50
        self.ht = 250
        self.k = 3
        self.iterations = 0

        #Define corners of 3D Model's bounding cube
        self.corners3D = np.zeros((3,5))
        tube_length = 1 # m
        tube_radius = 2.5 # m
        self.corners3D[0] = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.corners3D[1] = np.array([0.0, tube_radius, 0.0, -tube_radius, 0.0])
        self.corners3D[2] = np.array([0.0, 0.0, tube_radius, 0.0, -tube_radius])
        self.corners3D = np.transpose(self.corners3D)
        #print(self.corners3D)

        # camera intrinsic matrix
        self.K = np.zeros((3,3), dtype='float64')
        self.K[0, 0], self.K[0, 2] = 241.42682359130833, 376.5
        self.K[1, 1], self.K[1, 2] = 241.42682359130833, 240.5
        self.K[2, 2] = 1.     

        self.distCoeffs = np.zeros((8, 1), dtype='float32')  

        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]]).reshape(-1,3) 

        if(mode == "test"):

            self.fig = plt.figure(figsize=(20,10))
            self.ax = self.fig.add_axes([0.01, 0.25,0.45, 0.45])
            self.ax2 = self.fig.add_axes([0.50, 0.25, 0.45, 0.45])

            #create sliders
            axcolor = 'lightgoldenrodyellow'
            axcannylt = self.fig.add_axes([0.1, 0.05, 0.8, 0.01], facecolor= axcolor)
            axcannyht =self.fig.add_axes([0.1, 0.07, 0.8, 0.01], facecolor= axcolor)
            axiter = self.fig.add_axes([0.1, 0.09, 0.8, 0.01], facecolor= axcolor)
            axcannyk = self.fig.add_axes([0.1, 0.11, 0.8, 0.01], facecolor= axcolor)
            #axminRadius = self.fig.add_axes([0.1, 0.13, 0.8, 0.01], facecolor= axcolor)
            #axmaxRadius = self.fig.add_axes([0.1, 0.15, 0.8, 0.01], facecolor= axcolor)

            self.canny_lt_slider = Slider(axcannylt, 'canny_lt', 50, 500.0, valinit=self.lt)
            self.canny_ht_slider = Slider(axcannyht, 'canny_ht', 50, 500.0, valinit=self.ht)
            self.iter_slider = Slider(axiter, 'iter', 0.0, 10, valinit=0, valstep=self.iterations)
            self.canny_k_slider = Slider(axcannyk, 'canny K', 0.0, 30, valinit = self.k, valstep=2)
            #self.minRadius_slider = Slider(axminRadius, '*', 0.0,100,valinit=50)
            #self.maxRadius_slider = Slider(axmaxRadius, '*',0.0,100,valinit=70)

            self.canny_lt_slider.on_changed(self.update)
            self.canny_ht_slider.on_changed(self.update)
            self.iter_slider.on_changed(self.update)
            self.canny_k_slider.on_changed(self.update)
            #self.minRadius_slider.on_changed(self.update)
            #self.maxRadius_slider.on_changed(self.update)

    def update(self, val):
        """
        Read slider values and update algorithms' parameters for ellipse 3D pose
        estimation. The argument is ignored.
        """
        self.lt = int(self.canny_lt_slider.val)
        self.ht = int(self.canny_ht_slider.val)
        self.k = int(self.canny_k_slider.val)
        self.iterations = int(self.iter_slider.val)

        ellipses = self.detect(self.img.copy(), self.max_size)

        img = self.img.copy()
        for ellipse in ellipses:
            R, t, R_exp, e = ellipse
            img = cv2.ellipse(img, e, (255,0,0), 2)
            img = self.draw_frame(img,(e[0][0],e[0][1]), R_exp, t)

        #self.ax.imshow(edges)
        self.ax2.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        print(" >> Image updated")
        return img

    def getPose(self, points_3D, points_2D):
        """
        Estimate ellipse 3D pose based on 2D points and 3D points correspondence
        Args:
            points_3D (np.array): 3D points of the object 
            points_2D (np.array): 2D points in the image plane
        Returns:
            R (np.array): 3x3 rotation matrix following the opencv camera convention
            t (np.array): 3x1 translation vector
        """
        assert points_2D.shape[0] == points_2D.shape[0], 'points 3D and points 2D must have same number of vertices'

        points_2D = np.ascontiguousarray(points_2D[:,:2]).reshape((-1,1,2))

        _, R_exp, t = cv2.solvePnP(points_3D, points_2D, self.K, self.distCoeffs)
        R, _ = cv2.Rodrigues(R_exp)
        return R, t, R_exp

    def detect(self, img, max_size):

        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale)

        #gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        img  = cv2.GaussianBlur(img, (5,5),0) 
        edges = cv2.Canny(img, self.lt, self.ht, self.k)
        dilate = cv2.dilate(edges, None, iterations = self.iterations)
        erode = cv2.erode(dilate, None, iterations = self.iterations)
        cnts = cv2.findContours(erode, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        
        detected_ellipses = []
        for i, c in enumerate(cnts[1]):
            if c.shape[0] > 5:

                e = cv2.fitEllipse(c)
                # threshold by size 
                ex = int(e[0][0])
                ey = int(e[0][1])
                width = e[1][0]
                height = e[1][1]

                # filter ellipse by size
                if((width > 20 ) and (height > 20)):           

                    edge_arcLength = cv2.arcLength(c,False)
                    edge = c.reshape(-1,2)


                    empty = np.zeros_like(edges)
                    empty = cv2.ellipse(empty, e, (255,0,0), 2)
                    ellipse_edges = cv2.Canny(empty, self.lt, self.ht, self.k)
                    ellipse_cnts = cv2.findContours(ellipse_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if (len(ellipse_cnts[1]) == 1):
                        ellipse_edge = ellipse_cnts[1][0]
                        ellipse_edge_arclength = cv2.arcLength(ellipse_edge, True)
                        ellipse_edge = ellipse_edge.reshape(-1,2)
                    else:
                        # only 1 countours should be found, if not, continue
                        continue 

                    # filter by arclength
                    try:
                        coverage = edge_arcLength / ellipse_edge_arclength
                    except ZeroDivisionError:
                        continue

                    if ( (coverage > 0.8) and (coverage < 1.1)):
                        pass
                    else:
                        continue         

                    # compute centroids
                    edge_moments = cv2.moments(c)
                    try:
                        edgex = int(edge_moments["m10"] / edge_moments["m00"])
                        edgey = int(edge_moments["m01"] / edge_moments["m00"])
                    except ZeroDivisionError:
                        continue

                    edge_centroid = np.array([edgex, edgey])
                    ellipse_centroid = np.array([ex, ey])
                    centroids_distance =  np.linalg.norm(edge_centroid-ellipse_centroid)

                    # filter by centroid distance
                    if(centroids_distance < self.centroid_threshold):
                        pass                  
                    else:
                        continue                        

                    # get 2d points correspondences
                    el = Ellipse(a=width/2, b=height/2, cx=ex, cy=ey, theta = e[2]*np.pi/180.0)
                    corners2D = np.zeros((5,2), dtype='float32')
                    corners2D[0][0] = ex
                    corners2D[0][1] = ey
                    for i ,angle in enumerate([0.0, np.pi/2, np.pi, 3*np.pi/2]):
                        x,y = el.get_ellipse_point(angle)
                        corners2D[i+1][0] = x
                        corners2D[i+1][1] = y
                        cv2.circle(img, (int(x), int(y)), 4, (0, 0, 255), -1) 

                    
                    # get Rotation and translation
                    R, t, R_exp = self.getPose(self.corners3D, corners2D)

                    #rescale
                    a = int(e[0][0]*(1.0/scale)) 
                    b = int(e[0][1]*(1.0/scale))
                    c = int(e[1][0]*(1.0/scale))
                    d = int(e[1][1]*(1.0/scale))
                    e = ((a,b),(c,d), e[2])

                    detected_ellipses.append((R,t,R_exp,e))

        return detected_ellipses

    def draw_frame(self, img, corner, R_exp, t):
        """
        Draw over the image a coordinate frame with origin in corner 
        and axis extending to the points in imgpts.
        Args:
            img (np.array): image over which to draw the coordinate frame
            corner (tuple): tuple of coordinates of the center, (x,y)
            imgpts (np.array): coordinates of the other end of each of the three
                               axis

        Taken from: https://docs.opencv.org/master/d7/d53/tutorial_py_pose.html
        """
        #corner = tuple(corners[0].ravel())
        imgpts, jac = cv2.projectPoints(self.axis, R_exp, t, self.K, self.distCoeffs)

        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255,0,0), 2)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 2)
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0,0,255), 2)
        return img

def main(args):
    """
    Load image, set parameter sliders, show result
    """
    image = cv2.imread(args.image)
    mem = EllipseDetector()
    mem.img = image
    mem.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)

