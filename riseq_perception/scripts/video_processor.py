
import cv2
import sys, os
import argparse
from net_detector import detect_net
from ellipse_detector import detect_ellipse
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
    out = cv2.VideoWriter('np5.avi',fourcc, 30.0, (460,259))

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
        t2 = time.time()

        # draw net
        frame[net_mask != 0] = [0,0,255]

        # draw ellipses
        if ellipses is not None:
            for ellipse in ellipses:
                R,t,e = ellipse
                frame = cv2.ellipse(frame, e, (0,255,0), 1)

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