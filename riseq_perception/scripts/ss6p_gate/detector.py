# RISE-Q Team's Test 2 submission
#

import numpy as np
import cv2
import json

# import RISE-Q's code required libraries
import os
import time
import torch
from torch.autograd import Variable
from torchvision import transforms
from PIL import Image

# import network and utilities
from darknet import Darknet
from utils import *
from MeshPly import MeshPly


class Line():
    """
        Class for representing a line
    """
    def __init__(self,p1,p2):
        
        # slope
        if( (p2[0]-p1[0]) == 0.0 ):
            self.m = "NaN"     # vertical line
        else:
            self.m = (p2[1]-p1[1])/(p2[0]-p1[0])
        
        # intercept
        if(self.m == "NaN"):
            self.b = "NaN"
        else:
            self.b = -1.0*self.m*p1[0] + p1[1]

        self.p = p1   #store one sample

    def eval(self,x):
        # TODO verify if line is vertical
        return(x*self.m + self.b)

# Given 2 straight lines, return their intersection point
def find_intersection(l1, l2):
    x = (l2.b - l1.b)/(l1.m - l2.m) # x coord of intersection point
    y = l1.eval(x) # y coord of intersection point
    return x,y


class GateDetector():
    def __init__(self):


        # Parse configuration files
        self.options    = read_data_cfg('cfg/gate3.data')
        self.meshname   = self.options['mesh']
        self.name       = self.options['name']

        # Parameters for network
        self.seed = int(time.time())
        self.gpus = '0'                     # define gpus to use
        torch.manual_seed(self.seed)            # seed torch random
        self.test_width  = 416      # define test image size
        self.test_height = 416
        self.use_cuda = True
        if self.use_cuda:
            os.environ['CUDA_VISIBLE_DEVICES'] = self.gpus
            torch.cuda.manual_seed(self.seed)   # seed cuda random
        self.conf_thresh = 0.1
        self.num_classes = 1        

        # Read object 3D model, get 3D Bounding box corners
        #self.mesh = MeshPly(self.meshname)
        #self.vertices = np.c_[np.array(self.mesh.vertices), np.ones((len(self.mesh.vertices), 1))].transpose()
        #self.corners3D = get_3D_corners(self.vertices)
        self.diam = float(self.options['diam']) 

        #Define corners of 3D Model's bounding cube
        self.corners3D = np.zeros((4,8))
        feet_cm = 30.48 # 1 ft = 30.48 cm
        self.corners3D[0] = np.array([-11*feet_cm/2.0, -11*feet_cm/2.0, -11*feet_cm/2.0, -11*feet_cm/2.0, 11*feet_cm/2.0, 11*feet_cm/2.0, 11*feet_cm/2.0, 11*feet_cm/2.0])
        self.corners3D[1] = np.array([-feet_cm/2.0, -feet_cm/2.0, feet_cm/2.0, feet_cm/2.0, -feet_cm/2.0, -feet_cm/2.0, feet_cm/2.0, feet_cm/2.0])
        self.corners3D[2] = np.array([-11*feet_cm/2.0, 11*feet_cm/2.0, -11*feet_cm/2.0, 11*feet_cm/2.0, -11*feet_cm/2.0, 11*feet_cm/2.0, -11*feet_cm/2.0, 11*feet_cm/2.0])
        self.corners3D[3] = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # Calculate gate dimensions
        min_x = np.min(self.corners3D[0,:])      # this are the gate outermost corners
        max_x = np.max(self.corners3D[0,:])
        min_y = np.min(self.corners3D[1,:])
        max_y = np.max(self.corners3D[1,:])
        min_z = np.min(self.corners3D[2,:])
        max_z = np.max(self.corners3D[2,:])

        self.gate_dim_z = max_z - min_z
        self.gate_dim_x = max_x - min_x
        self.gate_dim_y = max_y - min_y

        # Calculate Fly are based based on offset from predicted 2D
        # Projection
        self.flyarea_side = 243.84 #cm
        offset_z = (self.gate_dim_z - self.flyarea_side)/2.0
        offset_x = (self.gate_dim_x - self.flyarea_side)/2.0

        self.offset_z_ratio = (offset_z/self.gate_dim_z)  # calculate as ratio wrt side, to use with pixels later
        self.offset_x_ratio = (offset_x/self.gate_dim_x)
        #print("Offset X ratio: {}, Offset Z ratio: {}".format(offset_x_ratio,offset_z_ratio))


        # now configure camera intrinsics
        self.internal_calibration = get_camera_intrinsic()

        # Create the network based on cfg file
        self.model = Darknet('cfg/yolo-pose.cfg')
        #self.model.print_network()
        self.model.load_weights('weights/model.weights')
        self.model.cuda()
        self.model.eval()

    def predict(self,img):

        # define test image size
        #test_width = img.shape[0]
        #test_height = img.shape[1]
        
        # Now prepare image: convert to RGB, resiz  e, transform to Tensor
        # use cuda, 
        img_copy = img.copy()
        img = Image.fromarray(img)
        ori_size = img.size         # store original size
        img = img.resize((self.test_width, self.test_height))
        img = transforms.Compose([transforms.ToTensor(),])(img)#.float()
        img = Variable(img, requires_grad = True)
        img = img.unsqueeze(0)
        img = img.cuda()

        # Forward pass
        output = self.model(img).data

        # Using confidence threshold, eliminate low-confidence predictions
        # and get only boxes over the confidence threshold
        all_boxes = get_region_boxes(output, self.conf_thresh, self.num_classes) 
        boxes = all_boxes[0]

        # iterate through boxes to find the one with highest confidence
        best_conf_est = -1
        best_box_index = -1
        for j in range(len(boxes)):
            # the confidence is in index = 18
            if( boxes[j][18] > best_conf_est):
                box_pr = boxes[j] # get bounding box
                best_conf_est = boxes[j][18]
                best_box_index = j
        # Denormalize the corner prediction
        # This are the predicted 2D points with which a bounding cube can be drawn
        corners2D_pr = np.array(np.reshape(box_pr[:18], [9, 2]), dtype='float32')
        corners2D_pr[:, 0] = corners2D_pr[:, 0] * ori_size[0]  # Width
        corners2D_pr[:, 1] = corners2D_pr[:, 1] * ori_size[1]  # Height
        # get rotation matrix and transform
        R_pr, t_pr = pnp(np.array(np.transpose(np.concatenate((np.zeros((3, 1)), self.corners3D[:3, :]), axis=1)), dtype='float32'),  corners2D_pr, np.array(self.internal_calibration, dtype='float32'))

        # Get each predicted point and the centroid
        p1 = corners2D_pr[1]
        p2 = corners2D_pr[2]
        p3 = corners2D_pr[3]
        p4 = corners2D_pr[4]
        p5 = corners2D_pr[5]
        p6 = corners2D_pr[6]
        p7 = corners2D_pr[7]
        p8 = corners2D_pr[8]
        center = corners2D_pr[0] 

        # Draw cube lines around detected object
        # draw front face
        line_point = 3
        cv2.line(img_copy,(p1[0],p1[1]),(p2[0],p2[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p2[0],p2[1]),(p4[0],p4[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p4[0],p4[1]),(p3[0],p3[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p3[0],p3[1]),(p1[0],p1[1]), (0,255,0),line_point)
        
        # draw back face
        cv2.line(img_copy,(p5[0],p5[1]),(p6[0],p6[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p7[0],p7[1]),(p8[0],p8[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p6[0],p6[1]),(p8[0],p8[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p5[0],p5[1]),(p7[0],p7[1]), (0,255,0),line_point)

        # draw right face
        cv2.line(img_copy,(p2[0],p2[1]),(p6[0],p6[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p1[0],p1[1]),(p5[0],p5[1]), (0,255,0),line_point)
        
        # draw left face
        cv2.line(img_copy,(p3[0],p3[1]),(p7[0],p7[1]), (0,255,0),line_point)
        cv2.line(img_copy,(p4[0],p4[1]),(p8[0],p8[1]), (0,255,0),line_point)

        # write confidence in top left corner
        cv2.putText(img_copy,"{:.2f}".format(best_conf_est.item()),(0,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2,cv2.LINE_AA)

        """

        ############################################################
        #       PREDICT FLYABLE AREA BASED ON ESTIMATED 
        #       2D PROJECTIONS OF 3D CORNERS
        ############################################################

        # Get each predicted point and the centroid
        p1 = corners2D_pr[1]
        p2 = corners2D_pr[2]
        p3 = corners2D_pr[3]
        p4 = corners2D_pr[4]
        p5 = corners2D_pr[5]
        p6 = corners2D_pr[6]
        p7 = corners2D_pr[7]
        p8 = corners2D_pr[8]
        center = corners2D_pr[0] 

        #           GATE'S FRONT FACE BOUNDING BOX CORNERS
        #
        # array to store all 4 points
        flyarea_corners_front = np.zeros((4,2), dtype = 'float32')
        # corner 1
        flyarea_corners_front[0][0] = p4[0] + int((p2[0]-p4[0])*self.offset_x_ratio)
        flyarea_corners_front[0][1] = p4[1] + int((p3[1]-p4[1])*self.offset_z_ratio)
        # corner 2
        flyarea_corners_front[1][0] = p2[0] + int((p4[0]-p2[0])*self.offset_x_ratio)
        flyarea_corners_front[1][1] = p2[1] + int((p1[1]-p2[1])*self.offset_x_ratio) 
        # corner 3
        flyarea_corners_front[2][0] = p1[0] + int((p3[0]-p1[0])*self.offset_x_ratio)
        flyarea_corners_front[2][1] = p1[1] + int((p2[1]-p1[1])*self.offset_x_ratio)
        # corner 4
        flyarea_corners_front[3][0] = p3[0] + int((p1[0]-p3[0])*self.offset_x_ratio)
        flyarea_corners_front[3][1] = p3[1] + int((p4[1]-p3[1])*self.offset_x_ratio) 

        #           GATE'S BACK FACE BOUNDING BOX CORNERS
        #      
        # array to store all 4 points
        flyarea_corners_back = np.zeros((4,2), dtype = 'float32')
        # corner 1
        flyarea_corners_back[0][0] = p8[0] + int((p6[0]-p8[0])*self.offset_x_ratio)
        flyarea_corners_back[0][1] = p8[1] + int((p7[1]-p8[1])*self.offset_z_ratio)
        # corner 2
        flyarea_corners_back[1][0] = p6[0] + int((p8[0]-p6[0])*self.offset_x_ratio)
        flyarea_corners_back[1][1] = p6[1] + int((p5[1]-p6[1])*self.offset_x_ratio) 
        # corner 3
        flyarea_corners_back[2][0] = p5[0] + int((p7[0]-p5[0])*self.offset_x_ratio)
        flyarea_corners_back[2][1] = p5[1] + int((p6[1]-p5[1])*self.offset_x_ratio)
        # corner 4
        flyarea_corners_back[3][0] = p7[0] + int((p5[0]-p7[0])*self.offset_x_ratio)
        flyarea_corners_back[3][1] = p7[1] + int((p8[1]-p7[1])*self.offset_x_ratio)         
            
        #           GATE FRONT FACE BOUNDING BOX EDGES 
        front_up = Line(flyarea_corners_front[0],flyarea_corners_front[1])
        front_right = Line(flyarea_corners_front[1],flyarea_corners_front[2])
        front_down = Line(flyarea_corners_front[2],flyarea_corners_front[3])
        front_left = Line(flyarea_corners_front[3],flyarea_corners_front[0])

        # BACK
        back_up = Line(flyarea_corners_back[0],flyarea_corners_back[1])
        back_right = Line(flyarea_corners_back[1],flyarea_corners_back[2])
        back_down = Line(flyarea_corners_back[2],flyarea_corners_back[3])
        back_left = Line(flyarea_corners_back[3],flyarea_corners_back[0])

        #   Identify innermost edges... they bound the flyable area
        if( back_up.p[1] > front_up.p[1]):
            up_line = back_up
        else:
            up_line = front_up
        if( back_down.p[1] < front_down.p[1]):
            down_line = back_down
        else:
            down_line = front_down
        if( back_right.p[0] < front_right.p[0]):
            right_line = back_right
        else:
            right_line = front_right
        if( back_left.p[0] > front_left.p[0] ):
            left_line = back_left
        else:
            left_line = front_left

        flyarea_corners = np.zeros((4,2), dtype = 'float32')  

        #   Extract the 4 points that define the flyable area
        x1,y1 = find_intersection(up_line,left_line)
        dummy1 = np.array([x1,y1])
        flyarea_corners[0] = dummy1
        x1,y1 = find_intersection(up_line,right_line)
        dummy1 = np.array([x1,y1])
        flyarea_corners[1] = dummy1
        x1,y1 = find_intersection(down_line,right_line)
        dummy1 = np.array([x1,y1])
        flyarea_corners[2] = dummy1
        x1,y1 = find_intersection(down_line,left_line)
        dummy1 = np.array([x1,y1])
        flyarea_corners[3] = dummy1

        # check if there was a gate or not
        if(best_conf_est.item() > self.conf_thresh):
            result = np.array([flyarea_corners.flatten()]).tolist()
            result[0].append(best_conf_est.item()) # add confidence
        else:
            result = np.array([np.array([]).flatten()]).tolist()
        """
        return R_pr, t_pr, img_copy, best_conf_est.item()
        
