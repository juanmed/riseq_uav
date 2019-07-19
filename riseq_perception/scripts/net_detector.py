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

class Memory():

    def __init__(self, max_size = 800):
        self.max_size = max_size
        self.img = 0

        self.fig = plt.figure(figsize=(20,10))
        self.ax = self.fig.add_axes([0.03, 0.25,0.45, 0.45])
        self.ax2 = self.fig.add_axes([0.50, 0.25, 0.45, 0.45])

        #create sliders
        axcolor = 'lightgoldenrodyellow'
        axcannylt = self.fig.add_axes([0.1, 0.05, 0.8, 0.01], facecolor= axcolor)
        axcannyht =self.fig.add_axes([0.1, 0.07, 0.8, 0.01], facecolor= axcolor)
        axiter = self.fig.add_axes([0.1, 0.09, 0.8, 0.01], facecolor= axcolor)
        axcannyk = self.fig.add_axes([0.1, 0.11, 0.8, 0.01], facecolor= axcolor)

        self.canny_lt_slider = Slider(axcannylt, 'canny_lt', 50, 500.0, valinit=70)
        self.canny_ht_slider = Slider(axcannyht, 'canny_ht', 50, 500.0, valinit=103)
        self.iter_slider = Slider(axiter, 'iter', 0.0, 10, valinit=4, valstep=1)
        self.canny_k_slider = Slider(axcannyk, 'canny K', 0.0, 30, valinit = 3, valstep=2)
        #self.minRadius_slider = Slider(axminRadius, '*', 0.0,100,valinit=50)
        #self.maxRadius_slider = Slider(axmaxRadius, '*',0.0,100,valinit=70)

        self.canny_lt_slider.on_changed(self.update)
        self.canny_ht_slider.on_changed(self.update)
        self.iter_slider.on_changed(self.update)
        self.canny_k_slider.on_changed(self.update)
        #self.minRadius_slider.on_changed(self.update)
        #self.maxRadius_slider.on_changed(self.update)        

    def update(self,val):
        """
        Net detection pipeline followed:
        https://docs.opencv.org/3.3.1/d3/db4/tutorial_py_watershed.html
        """
        lt = int(self.canny_lt_slider.val)
        ht = int(self.canny_ht_slider.val)
        k = int(self.canny_k_slider.val)
        iterations = int(self.iter_slider.val)


        gray, self.img = self.preprocess(self.img.copy())
        #proc = cv2.GaussianBlur(self.img.copy(), (5,5),0) 
        #proc = cv2.medianBlur(img.copy(), 5)   
        img = self.img.copy()
        t1 = time.time()
        edges = cv2.Canny(img.copy(), lt, ht, k)

        kernel = 5
        sigma = 1
        theta = 0
        lambd = 0.1
        gamma = 0.02
        psi = 0
        gabor_kernel = cv2.getGaborKernel((kernel, kernel), sigma, theta, lambd, gamma, psi)
        img = cv2.filter2D(edges, -1, gabor_kernel) #cv2.CV_8U

        ret, thresh = cv2.threshold(img,0,255,cv2.THRESH_OTSU) #cv2.THRESH_BINARY_INV+

        # noise removal
        kernel = np.ones((5,5),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

        ret, markers = cv2.connectedComponents(opening)
        markers = markers + 1
        #print(np.unique(markers))

        t2 = time.time()
        print(" Execution time: {}".format(t2-t1))
        scale = np.int(255/np.max(markers)) 
        markers = markers*scale

        #print(markers.shape)

        marker_size = []
        for i, marker in enumerate(np.unique(markers).tolist()):
            temp = np.zeros_like(markers)
            temp[markers == marker] = 1
            cumsum = temp.flatten().sum()    
            marker_size.append((marker, cumsum))
        
        print(marker_size)
        marker_size.sort(key= (lambda x: x[1]), reverse=True) 
        print(marker_size)

        temp = self.img.copy()
        temp[markers == marker_size[1][0]] = [0,0,255]

        #
        #print(np.unique(markers))

        #markers = markers+1
        #temp = self.img.copy()
        #markers = cv2.watershed(temp,markers)
        #temp[markers == -1] = [255,0,255]

        # erode, dilate
        #img = cv2.erode(img, None, iterations = iterations)

        # averaging filter
        #kernel = np.ones((10,10),np.float32)/100
        #img = cv2.filter2D(img,-1,kernel)


        #ft = np.fft.fft2(edges)
        #ft_mag = np.abs(ft)
        #ft_mag_log = 20*np.log(ft_mag)

        self.ax.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
        self.ax2.imshow(markers, cmap = 'jet')#cv2.cvtColor(temp, cv2.COLOR_BGR2RGB))#)
        self.ax2.imshow(cv2.cvtColor(temp, cv2.COLOR_BGR2RGB))


    def preprocess(self, img):
        """
        Resize, blur and convert input image to grayscale
        Args:
            img (np.array): input image to preprocess with 3 channels
        Returns:
            gray (np.array): grayscale image, 1 channel
            img (np.array): resized and possibly blured img, 3 channels
        """
        scale = self.max_size / float(max(img.shape))
        img = cv2.resize(img, None, fx=scale, fy = scale) 

        if(img.shape[2] == 3):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        return gray, img

# Gabor filter parameter
kernel = 5
sigma = 1
theta = 0
lambd = 0.1
gamma = 0.02
psi = 0
gabor_kernel = cv2.getGaborKernel((kernel, kernel), sigma, theta, lambd, gamma, psi)
o_kernel = np.ones((5,5),np.uint8)

def detect_net(img, max_size):
    """
    Resize image, detect edges, pass through gabor filter, threshold, do
    morphological opening, get connected components, get the largest
    group of connected components and return mask
    """
    scale = max_size / float(max(img.shape))
    # Canny parameters
    lt = 70
    ht = 100
    k = 3


    #img = cv2.resize(img, None, fx=scale, fy = scale) 
    img = cv2.Canny(img, lt, ht, k)
    img = cv2.filter2D(img, -1, gabor_kernel)
    ret, img = cv2.threshold(img,0,255,cv2.THRESH_OTSU)
    img = cv2.morphologyEx(img,cv2.MORPH_OPEN, o_kernel, iterations = 2)
    ret, markers = cv2.connectedComponents(img)

    marker_size = []
    for i, marker in enumerate(np.unique(markers).tolist()):
        temp = np.zeros_like(markers)
        temp[markers == marker] = 1
        cumsum = temp.flatten().sum()    
        marker_size.append((marker, cumsum))    
    marker_size.sort(key= (lambda x: x[1]), reverse=True) 
    
    temp = np.zeros_like(img)
    if (len(marker_size) > 2):
        temp[markers == marker_size[1][0]] = 255
    return temp

def main(args):
    """
    """
    image = cv2.imread(args.image)
    mem = Memory()
    mem.img = image
    mem.update(1)
    plt.show()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)