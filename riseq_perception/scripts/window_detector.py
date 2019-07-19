import os
import time
import numpy as np 
import argparse
import time

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import cv2

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

class WindowDetector():

    def __init__(self, mode="test"):
        """
        Initialize parameters. Mode must be 'test' if you want to display the 
        result with sliders that update parameters of each algorithm.
        """
        self.max_size = 460
        self.img = 0
        self.window_width = 1.4 #m
        self.window_height = 2.2 #m 

        # This params must be initialized with the best performing values
        self.canny_lt = 75
        self.canny_ht = 200
        self.canny_k = 5
        self.epsilon = 0.1  # should be between 0.02 to 0.15
        self.gauss_k = 5    # should be 3 or 5, but no more

        #Define corners of 3D Model's bounding cube
        self.corners3D = np.zeros((4,3))
        self.corners3D[0] = np.array([0.0, self.window_width/2, self.window_height/2])
        self.corners3D[1] = np.array([0.0, -self.window_width/2, self.window_height/2])
        self.corners3D[2] = np.array([0.0, -self.window_width/2, -self.window_height/2])
        self.corners3D[3] = np.array([0.0, self.window_width/2, -self.window_height/2]) 
        #self.corners3D[4] = np.array([0.0, 0.0, 0.0]) #center

        # camera intrinsic matrix
        self.K = np.zeros((3,3), dtype='float64')
        self.K[0, 0], self.K[0, 2] = 241.42682359130833, 376.5
        self.K[1, 1], self.K[1, 2] = 241.42682359130833, 240.5
        self.K[2, 2] = 1.     
        self.distCoeffs = np.zeros((8, 1), dtype='float32')  
        self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]]).reshape(-1,3) 
               
        if (mode == "test"):
            self.fig = plt.figure(figsize=(20,10))
            self.ax = self.fig.add_axes([0.03, 0.25,0.45, 0.45])
            self.ax2 = self.fig.add_axes([0.50, 0.25, 0.45, 0.45])

            #create sliders
            axcolor = 'lightgoldenrodyellow'
            axcannylt = self.fig.add_axes([0.1, 0.05, 0.8, 0.01], facecolor= axcolor)
            axcannyht =self.fig.add_axes([0.1, 0.07, 0.8, 0.01], facecolor= axcolor)
            axgaussk = self.fig.add_axes([0.1, 0.09, 0.8, 0.01], facecolor= axcolor)
            axcannyk = self.fig.add_axes([0.1, 0.11, 0.8, 0.01], facecolor= axcolor)
            axepsilon = self.fig.add_axes([0.1, 0.13, 0.8, 0.01], facecolor= axcolor)

            self.canny_lt_slider = Slider(axcannylt, 'canny_lt', 50, 500.0, valinit=75)
            self.canny_ht_slider = Slider(axcannyht, 'canny_ht', 50, 500.0, valinit=200)
            self.gauss_k_slider = Slider(axgaussk, 'Gaussian K', 3, 15, valinit=5, valstep=2)
            self.canny_k_slider = Slider(axcannyk, 'canny K', 3, 30, valinit = 5, valstep=2)
            self.epsilon_slider = Slider(axepsilon, 'epsilon', 0.02, 1.0, valinit=0.1)
            #self.maxRadius_slider = Slider(axmaxRadius, '*',0.0,100,valinit=70)

            self.canny_lt_slider.on_changed(self.update)
            self.canny_ht_slider.on_changed(self.update)
            self.gauss_k_slider.on_changed(self.update)
            self.canny_k_slider.on_changed(self.update)
            self.epsilon_slider.on_changed(self.update)
            #self.maxRadius_slider.on_changed(self.update)              

    def update(self, val):
        """
        """
        self.canny_lt = int(self.canny_lt_slider.val)
        self.canny_ht = int(self.canny_ht_slider.val)
        self.canny_k = int(self.canny_k_slider.val)
        self.epsilon = self.epsilon_slider.val
        self.gauss_k = int(self.gauss_k_slider.val)

        R, t, R_exp, cnt = self.detect(self.img.copy(), self.max_size)
        #print("Rotation:\n{}\ntranslation:\n{}".format(R,t))

        if(cnt is not None):
            img = self.draw_frame(self.img.copy(),(cnt[0][0],cnt[0][1]), R_exp, t)
            img = cv2.drawContours(img, [cnt[1:]], -1, (255,0,0), 3)
        else:
            print("No Countours found")
            img = self.img
            #raise RunTimeError("No contours were found.")
        
        #self.ax.imshow(img)
        self.ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    def detect(self, img, max_size):
        """
        """
        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale) 

        if(img.shape[2] == 3):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        blur = cv2.GaussianBlur(gray, (self.gauss_k, self.gauss_k), 0)
        #blur = cv2.bilateralFilter(gray, self.gauss_k, 7, 7, borderType=cv2.BORDER_REPLICATE)
        edges = cv2.Canny(blur.copy(), self.canny_lt, self.canny_ht, self.canny_k)
        dilate = cv2.dilate(edges, None, iterations = 1)
        erode = cv2.erode(dilate, None, iterations = 1)
        inf, cnts, hrch = cv2.findContours(erode.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cnts_tree = cnts.copy()
        try:
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
        except:
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:len(cnts)]

        screenCnt = None
        R = None
        t = None
        R_exp = None
        corners2D = None

        squares = []

        for cnt in cnts:

            # approximate the contour
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.epsilon * peri, True)

            # if our approximated contour has four points, then we
            # can assume that we have found our screen
            if (len(approx) == 4):
                squares.append(approx)

        square_couples = []
        for i, square_i in enumerate(squares):

            m = cv2.moments(square_i)
            try:
                cx = int( m["m10"] / m["m00"] )
                cy = int( m["m01"] / m["m00"] )
            except ZeroDivisionError:
                continue

            centroid_i = np.array([[cx],[cy]])

            for j in range(len(squares)):

                if (i == j):
                    continue
                else:
                    square_j = squares[j]

                    m = cv2.moments(square_j)
                    try:
                        cx_j = int( m["m10"] / m["m00"] )
                        cy_j = int( m["m01"] / m["m00"] )
                    except ZeroDivisionError:
                        continue

                    centroid_j = np.array([[cx_j],[cy_j]])

                    dist = np.linalg.norm(centroid_i - centroid_j)
                    if dist < 10:
                        square_couples.append([square_i, square_j, dist])

        square_couples.sort(key=(lambda x: x[2]), reverse = True)

        if len(square_couples) > 0:
            screenCnt = square_couples[0][0].reshape((-1,2))
            screenCnt = (screenCnt*(1.0/scale)).astype('float32')
                

            screenCnt = screenCnt.reshape((4,-1))

            #screenCnt2 = np.sort(screenCnt, axis = 0)
            #screenCnt2 = np.sort(screenCnt2, axis = 1)
            #print(screenCnt2)

            #get centroid
            m = cv2.moments(screenCnt)
            try:
                cx = int( m["m10"] / m["m00"] )
                cy = int( m["m01"] / m["m00"] )
            except ZeroDivisionError:
                pass

            centroid = np.array([[cx,cy]])
            corners2D = screenCnt
            #corners2D = np.concatenate((screenCnt2, centroid), axis = 0)

            #print(self.corners3D, corners2D)
            R, t, R_exp = self.getPose(self.corners3D, corners2D)

            corners2D = np.concatenate((centroid, screenCnt), axis = 0)
            corners2D = corners2D.astype('int')
            

        return R, t, R_exp, corners2D

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
        assert points_3D.shape[0] == points_2D.shape[0], 'points 3D and points 2D must have same number of vertices'

        points_2D = np.ascontiguousarray(points_2D[:,:2]).reshape((-1,1,2))
        _, R_exp, t = cv2.solvePnP(points_3D, points_2D, self.K, None, flags = cv2.SOLVEPNP_EPNP)
        R, _ = cv2.Rodrigues(R_exp)
        return R, t, R_exp

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
    """
    image = cv2.imread(args.image)
    wd = WindowDetector(mode = "test")
    wd.img = image
    wd.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)