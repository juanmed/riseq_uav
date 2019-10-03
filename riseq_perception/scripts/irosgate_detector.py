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

class IROSGateDetector():

    def __init__(self, mode="test", color_space = 'BGR'):
        """
        Initialize parameters. Mode must be 'test' if you want to display the 
        result with sliders that update parameters of each algorithm.
        """
        self.max_size = 460
        self.img = 0
        self.window_width = 1.4 #m
        self.window_height = 1.4 #m 
        self.color_space = color_space

        # This params must be initialized with the best performing values
        self.canny_lt = 75
        self.canny_ht = 200
        self.canny_k = 5
        self.epsilon = 0.1  # should be between 0.02 to 0.15
        self.gauss_k = 5    # should be 3 or 5, but no more

        # For how to decide HSV color boundaries, look
        # https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
        self.HSVboundaries = [([160, 100, 40], [180, 255, 255]), #red
                              ([0, 100, 40], [30, 255, 255])]
                             #([25, 146, 190], [62, 174, 250]),
                             #([103, 86, 65], [145, 133, 128])]



        #Define corners of 3D Model's bounding cube
        self.corners3D = np.zeros((4,3))
        self.corners3D[0] = np.array([ self.window_width/2,  0.0, self.window_height/2])
        self.corners3D[1] = np.array([ -self.window_width/2, 0.0, self.window_height/2])
        self.corners3D[2] = np.array([ -self.window_width/2, 0.0, -self.window_height/2])
        self.corners3D[3] = np.array([ self.window_width/2,  0.0, -self.window_height/2]) 
        #self.corners3D[4] = np.array([0.0, 0.0, 0.0]) #center

        # camera intrinsic matrix
        self.K = np.zeros((3,3), dtype='float64')
        self.K[0, 0], self.K[0, 2] = 697.5170288085938, 638.0659790039062 #241.42682359130833, 376.5
        self.K[1, 1], self.K[1, 2] = 697.5170288085938, 354.1419982910156#241.42682359130833, 240.5
        self.K[2, 2] = 1.     
        self.distCoeffs = np.array([-0.17601299285888672, 0.028582999482750893, 0.0, 0.0006652449956163764, -0.0005063589778728783])# np.zeros((8), dtype='float32')  
        self.axis = np.float32([[0.5,0,0], [0,0.5,0], [0,0,-0.5]]).reshape(-1,3) 
               
        if (mode == "test"):
            self.color_space = 'BGR'
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

        R, t, R_exp, cnt, mask, cnts = self.detect(self.img.copy(), self.max_size)
        print("Translation: ",t)
        print("Rotation: ",R_exp)

        if(cnt is not None):
            img = self.draw_frame(self.img.copy(),(cnt[0][0],cnt[0][1]), R_exp, t)
            img = cv2.drawContours(img, [cnt[1:]], -1, (255,0,0), 3)
        else:
            print("No Countours found")
            img = self.img
            #raise RunTimeError("No contours were found.")

        output = cv2.bitwise_and(self.img.copy(), img, mask = mask)
        if (self.color_space == 'BGR'):
            self.ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            #self.ax2.imshow(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))
            self.ax2.imshow(cv2.cvtColor(cv2.drawContours(output, cnts, -1, (0,255,0), 2), cv2.COLOR_BGR2RGB))
        else:
            self.ax.imshow(img)
            self.ax2.imshow(output)            


    def detect(self, img, max_size):
        """
        """
        scale = 1.0
        #scale = self.max_size / float(max(img.shape))
        #img = cv2.resize(img, None, fx=scale, fy = scale) 

        toGray = False
        if toGray:
            if(img.shape[2] == 3):
                if(self.color_space == 'BGR'):
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                else:
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            else:
                pass

        # blur, filter colors, dilate and erode to define edges, and find contours
        blur = cv2.GaussianBlur(img, (self.gauss_k, self.gauss_k), 0)
        mask = self.filterColors(blur)
        mask = cv2.dilate(mask, None, iterations = 1)
        mask = cv2.erode(mask, None, iterations = 1)
        inf, cnts, hrch = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # get only the biggest contours, up to 10 contour elements
        try:
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
        except:
            pass

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
            # can assume that we have found our square
            if (len(approx) == 4):
                #@todo also sort corners by x,y position
                squares.append(approx)

        for square in squares:

            x,y,w,h = cv2.boundingRect(square)

            # verify width and height are similar
            aspect_ratio = w/h
            if(not ((aspect_ratio > 0.9) and (aspect_ratio < 1.1))):
                continue
            else:
                pass

            # verify area
            valid_area = self.verifyArea(w,h,square)
            if(not valid_area):
                continue
            else:
                pass
                

            screenCnt =  square.reshape((-1,2)) #square_couples[0][0].reshape((-1,2))
            screenCnt = (screenCnt*(1.0/scale)).astype('float32')
            screenCnt = screenCnt.reshape((4,-1))

            #get centroid
            m = cv2.moments(screenCnt)
            try:
                cx = int( m["m10"] / m["m00"] )
                cy = int( m["m01"] / m["m00"] )
                
                centroid = np.array([[cx,cy]])
                corners2D = screenCnt
                #corners2D = np.concatenate((screenCnt2, centroid), axis = 0)

                R, t, R_exp = self.getPose(self.corners3D, corners2D, scale)

                corners2D = np.concatenate((centroid, screenCnt), axis = 0)
                corners2D = corners2D.astype('int')

            except ZeroDivisionError:
                pass

                break

        return R, t, R_exp, corners2D, cv2.resize(mask, None, fx = 1.0/scale, fy = 1.0/scale), cnts

    def verifyArea(self, w,h,square):
        """
        """
        valid_area = False
        area1 = w*h
        area2 = cv2.contourArea(square)
        print("w*h : {}  contourArea: {}".format(area1, area2))
        if (((area1/area2) > 0.9) and ((area1/area2) < 1.1)): 
            valid_area = True
        return valid_area



    def filterColors(self, img):
        """
        Get an RGB image, transfor to HSV, and filter only specific colors
        defined by HSV boundaries
        Args:
            img: image to transfor to from RGB to HSV 
        Returns:
            mask: A 2-dimension boolean np.array whose elements are true
                  if the image pixel value in the same position of element is
                  inside the boundaries specified
        """

        # store here the final mask
        mask = np.zeros(img.shape[:2], dtype=bool)

        # Transfor to HSV color space
        if(self.color_space == 'BGR'):
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        for (lower, upper) in self.HSVboundaries:
            # create NumPy arrays from the HSVboundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
         
            # find the colors within the specified HSVboundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper) + mask
        
        return mask      


    def getPose(self, points_3D, points_2D, scale = 1.):
        """
        Estimate ellipse 3D pose based on 2D points and 3D points correspondence
        Args:
            points_3D (np.array): 3D points of the object 
            points_2D (np.array): 2D points in the image plane
            scale (int):factor by which the original image's width and height was modified
        Returns:
            R (np.array): 3x3 rotation matrix following the opencv camera convention
            t (np.array): 3x1 translation vector
        """
        assert points_3D.shape[0] == points_2D.shape[0], 'points 3D and points 2D must have same number of vertices'

        points_2D = np.ascontiguousarray(points_2D[:,:2]).reshape((-1,1,2))
        K_scaled = self.scale_intrinsic_matrix(self.K, scale)
        rvec_prior = cv2.Rodrigues(np.diag([1.0,1.0,1.0]))
        #print(rvec_prior)
        #rvec_prior = (rvec_prior[0][0], rvec_prior[1][0], rvec_prior[2][0])
        #tvec_prior = np.array([[0.0],[-1.0],[5.0]])
        _, R_exp, t = cv2.solvePnP(points_3D, points_2D, K_scaled, distCoeffs = self.distCoeffs, flags = cv2.SOLVEPNP_EPNP)
        R, _ = cv2.Rodrigues(R_exp)
        return R, t, R_exp

    def scale_intrinsic_matrix(self, K, scale):
        """
        If a pair (image, intrinsic matrix), and the image's width and height are 
        modified by a factor of 'scale', then the focal lenght and center point 
        are also modified by the same factor.
        See: https://komputervision.wordpress.com/2016/03/21/intrinsic-camera-parameters-for-resized-images-set-of-a-chessboard/
        """
        K = K * scale
        K[2, 2] = 1.
        return K

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
    wd = IROSGateDetector(mode = "test")
    wd.img = image
    wd.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)