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

class LadderDetector():

    def __init__(self, mode = "test"):
        self.max_size = 800
        self.img = 0

        self.boundaries = [([160, 100, 20], [180, 255, 255]), #red
                           ([0, 100, 20], [15, 255, 255])]
                           #([25, 146, 190], [62, 174, 250]),
                           #([103, 86, 65], [145, 133, 128])]

        # Canny parameters
        self.lt = 50
        self.ht = 250
        self.k = 3
        self.iterations = 0

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
        """
        self.lt = int(self.canny_lt_slider.val)
        self.ht = int(self.canny_ht_slider.val)
        self.k = int(self.canny_k_slider.val)
        self.iterations = int(self.iter_slider.val)

        bbox, outimg = self.detect(self.img.copy(), self.max_size) 
        print("Bbox",bbox)
        
        x,y,w,h = bbox 
        img = cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)
        img = cv2.circle(img, (int(x + w/2),(y + h/2)), 4, (0,255,0), -1)
        
        self.ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        self.ax2.imshow(cv2.cvtColor(outimg, cv2.COLOR_BGR2RGB))
        return bbox


    def detect(self, img, max_size):
        """
        """

        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)   

        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale)

        # loop over the boundaries
        for (lower, upper) in self.boundaries:
            # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
         
            # find the colors within the specified boundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper)
            output = cv2.bitwise_and(img, img, mask = mask)
            output = cv2.resize(output, None, fx = 1.0/scale, fy = 1.0/scale)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[1]
            center = None
         
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing bbox
                c = max(cnts, key=cv2.contourArea)
                bbox = [int(a/scale) for a in cv2.boundingRect(c)]
            else:
                bbox = None


        return bbox,output


def main(args):
    """
    Load image, set parameter sliders, show result
    """
    image = cv2.imread(args.image)
    mem = LadderDetector()
    mem.img = image
    mem.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)
