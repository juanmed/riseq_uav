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
import sys, os
import argparse
from net_detector import detect_net
from ellipse_detector import detect_ellipse
from window_detector import WindowDetector
import time



def build_arg_parser():
    """
    Build command line argument parser
    Return:
        Dictionary containing arguments and their values
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", required = True, help = "Path to the image")
    ap = ap.parse_args()
    return ap  


def main(args):

    max_size=460

    # create video capture, codec and storage
    cap = cv2.VideoCapture(args.video)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')    
    out = cv2.VideoWriter('~/np5.avi',fourcc, 30.0, (460,259))

    wd = WindowDetector(mode="eval")


    add = 1
    fps_sum = 0
    frame_count = 0
    ellipses = None
    while(cap.isOpened()):
        try:
            ret, frame = cap.read()
        except:
            add = 0
            break

        # process
        frame_count = frame_count + 1

        t1 = time.time()
        scale = max_size / float(max(frame.shape))
        frame = cv2.resize(frame, None, fx=scale, fy = scale)
        net_mask = detect_net(frame.copy(), max_size)
        ellipses = detect_ellipse(frame.copy(), max_size)
        R, t, R_exp, cnt = wd.detect(frame.copy(), max_size)
        t2 = time.time()

        # draw net
        if(net_mask is not None):
            frame[net_mask != 0] = [0,0,255]

        # draw ellipses
        if ellipses is not None:
            for ellipse in ellipses:
                R,t,e = ellipse
                frame = cv2.ellipse(frame, e, (0,255,0), 1)

        if cnt is not None:
            frame = cv2.drawContours(frame, [cnt[1:]], -1, (255,0,0), 3)


        # calculate avg fps
        fps = 1.0/(t2-t1)
        fps_sum = fps_sum+fps
        
        frame = cv2.putText(frame, "{:.1f}".format(fps_sum/frame_count), (max_size//40, max_size//20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        if add:
            out.write(frame)
            cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    out.release()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    args = build_arg_parser()
    main(args)