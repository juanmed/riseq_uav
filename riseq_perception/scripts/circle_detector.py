#!/usr/bin/env python

import cv2
import argparse
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.widgets import Slider

max_size = 460


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

def detect_circles(img, dp, minDist, param1, param2, minRadius, maxRadius):
    """
    Given an input grayscale image, convert to Grayscale and detect circles
    """

    #detect circles
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp = dp, minDist=minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

    if circles is not None:
        print circles

    return circles

def draw_circles(circles, img):
    """
    Draw circles over img. 
    Returns:
        img -- image with circles and their center drawn in red color
    """

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        print(circles)
        for (x, y, r) in circles:
            # draw the circles over img
            cv2.circle(img, (x, y), r, (0, 0, 255), 2)
            cv2.circle(img, (x,y),2,(0,0,255),2)
    return img
     
class Memory():

    def __init__(self, max_size = 460):
        self.max_size = max_size
        
        self.fig = plt.figure(figsize=(10,10))
        self.img = 0

        self.ax = self.fig.add_axes([0.1,0.25,0.8, 0.8])
        axcolor = 'lightgoldenrodyellow'
        axds = self.fig.add_axes([0.1, 0.05, 0.8, 0.01], facecolor= axcolor)
        axminDist =self.fig.add_axes([0.1, 0.07, 0.8, 0.01], facecolor= axcolor)
        axparam1 = self.fig.add_axes([0.1, 0.09, 0.8, 0.01], facecolor= axcolor)
        axparam2 = self.fig.add_axes([0.1, 0.11, 0.8, 0.01], facecolor= axcolor)
        axminRadius = self.fig.add_axes([0.1, 0.13, 0.8, 0.01], facecolor= axcolor)
        axmaxRadius = self.fig.add_axes([0.1, 0.15, 0.8, 0.01], facecolor= axcolor)

        dp_slider = Slider(axds, 'dp', 0.1, 30.0, valinit=1.2)
        minDist_slider = Slider(axminDist, 'minDist', 0.1, 200.0, valinit=100)
        param1_slider = Slider(axparam1, 'param1', 0.0, 200, valinit=100)
        param2_slider = Slider(axparam2, 'param2', 0.0, 200, valinit = 100)
        minRadius_slider = Slider(axminRadius, 'minRadius', 0.0,100,valinit=50)
        maxRadius_slider = Slider(axmaxRadius, 'maxRadius',0.0,100,valinit=70)

        dp_slider.on_changed(self.update)
        minDist_slider.on_changed(self.update)
        param1_slider.on_changed(self.update)
        param2_slider.on_changed(self.update)
        minRadius_slider.on_changed(self.update)
        maxRadius_slider.on_changed(self.update)

        self.sliders = [dp_slider, minDist_slider, param1_slider, param2_slider, minRadius_slider, maxRadius_slider]

    def update(self, val):
        """
        Get sliders values and update detected circles with new slider
        parameters
        """
        dp = self.sliders[0].val
        minDist = int(self.sliders[1].val) 
        param1 = int(self.sliders[2].val)
        param2 = int(self.sliders[3].val)
        minRadius = int(self.sliders[4].val)
        maxRadius = int(self.sliders[5].val)

        gray, self.img = self.preprocess(self.img.copy())
        circles = detect_circles(gray, dp, minDist, param1, param2, minRadius, maxRadius)
        output = draw_circles(circles, self.img.copy())

        self.ax.imshow(output)
        print(" >> Image updated")
        return output

    def preprocess(self, img):
        """
        Resize, blur and convert input image to grayscale
        """
        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale) 
        proc = cv2.medianBlur(img.copy(), 5)   

        if(proc.shape[2] == 3):
            gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
        else:
            gray = proc

        return gray, img

def main(args):
    """
    Load image, convert to Grayscale, and detect circles
    """
    image = cv2.imread(args.image)    

    mem = Memory()
    mem.img = image
    mem.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)