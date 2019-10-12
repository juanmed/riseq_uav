#!/usr/bin/env python

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

import rospy
import numpy as np
import tf
import cv2

#from detector import *
from window_detector import WindowDetector
from ellipse_detector import EllipseDetector
from ladder_detector import LadderDetector 
from irosgate_detector import IROSGateDetector
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MonoWaypointDetector():

    def __init__(self):
        """
        Initialize CNN for gate pose detection
        """

        self.waypoint_pub = rospy.Publisher("riseq/perception/uav_mono_waypoint", Path, queue_size = 10)
        self.img_dect_pub = rospy.Publisher("riseq/perception/uav_image_with_detections", Image, queue_size = 10)
        self.object_centerpoint_pub = rospy.Publisher("riseq/perception/uav_mono_waypoint2d", PoseStamped, queue_size = 10)        
        self.frontCamera_Mono = rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, self.estimate_object_pose)
        self.frontCamera_Mono_info = rospy.Subscriber("/zed/zed_node/left_raw/camera_info", CameraInfo, self.camera_params)
        #self.frontCamera_Mono = rospy.Subscriber("/iris/camera_nadir/image_raw", Image, self.estimate_object_pose)
        #self.frontCamera_Mono_info = rospy.Subscriber("/iris/camera_nadir/camera_info", CameraInfo, self.camera_params)
        self.bridge = CvBridge()

        self.mode = rospy.get_param("riseq/monocular_cv", 'disable')
        self.max_size = None    # 


        # AI Challenge detectors
        self.wd = WindowDetector(mode="eval")
        self.pd = EllipseDetector(mode = "eval")
        self.ld = LadderDetector(mode = "eval")
        self.ig = IROSGateDetector(mode = "eval", color_space = 'RGB')
        self.camera_info_ready = False
        self.camera_info_received = False   

        self.enable_recording = False
        self.frames = 0.
        self.success = 0.
        self.saved = False


        # ADR Gate Detector
        #cfg_file = rospy.get_param("riseq/gate_pose_nn_cfg")
        #ply_file = rospy.get_param("riseq/gate_pose_nn_ply")
        #wgt_file = rospy.get_param("riseq/gate_pose_nn_wgt")
        #data_file = rospy.get_param("riseq/gate_pose_nn_data")
        #self.gateDetector = GateDetector(cfg_file, ply_file, wgt_file, data_file)

    def estimate_object_pose(self, image_msg):
        """
        Receive an image, pass it through the network, and get the 
        estimated gate translation and rotation in camera frame.

        Args:
            image_msg: RGB image
        Returns:
            The detected translation as a 3-vector (np.array, [x,y,z]) and 
            orientation as a quaternion 4-vector(np.array, [x,y,z,w])
        """

        r = rospy.Rate(2)
        while not self.camera_info_received:
            rospy.loginfo("Camera Info not yet received. Waiting...")
            r.sleep()

        if not self.camera_info_ready:
            self.ig.K = self.K
            self.ig.D = self.D
            rospy.loginfo("Camera info set.")
            self.camera_info_ready = True
            self.initKalmanFilters()
            if(self.enable_recording):
                fourcc = cv2.VideoWriter_fourcc(*'XVID')    
                self.out = cv2.VideoWriter('gate_tracking.avi',fourcc, 20.0, (self.image_width,self.image_height)) 

        # update MonoWaypointDetector
        self.mode = rospy.get_param("riseq/monocular_cv", 'irosgate')
        self.frames = self.frames + 1.

        if(self.mode != 'disable'):

            img = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")

            if not self.saved and self.frames == 15.:
                cv2.imwrite(r"gatelow.jpg",cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                print("Image saved!")
                self.saved = True
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = ""

            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""

            wp2d = PoseStamped()
            wp2d.header.stamp = rospy.Time.now()
            wp2d.header.frame_id = ""

            if(self.mode == 'window'):

                R, t, R_exp, cnt = self.wd.detect(img.copy(), self.max_size)
                print("rvec: {}\ntvec: {}".format(R_exp, t))
                if cnt is not None:

                    img = cv2.drawContours(img, [cnt[1:]], -1, (255,0,0), 3)
                    img = self.wd.draw_frame(img, (cnt[0][0],cnt[0][1]), R_exp, t)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis = 0)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T ), axis = 1)
                    gate_quat = tf.transformations.quaternion_from_matrix(R)

                    # gate waypoint
                    x = t[2][0]
                    y = -t[0][0]
                    z = -t[1][0]

                    wp.pose.position.x = x
                    wp.pose.position.y = y
                    wp.pose.position.z = z
                    wp.pose.orientation.x = gate_quat[0]
                    wp.pose.orientation.y = gate_quat[1]
                    wp.pose.orientation.z = gate_quat[2]
                    wp.pose.orientation.w = gate_quat[3]

                    path.poses = [wp]

            elif(self.mode == 'pipe'):

                ellipses = self.pd.detect(img.copy(), self.max_size)
                
                if len(ellipses) == 1:
                    R, t, R_exp, e = ellipses[0]
                    img = cv2.ellipse(img, e, (255,0,0), 2)
                    img = self.pd.draw_frame(img, (e[0][0],e[0][1]), R_exp, t)

                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis = 0)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T ), axis = 1)
                    gate_quat = tf.transformations.quaternion_from_matrix(R)

                    # gate waypoint
                    wp.pose.position.x = t[2][0]
                    wp.pose.position.y = -t[0][0]
                    wp.pose.position.z = t[1][0]
                    wp.pose.orientation.x = gate_quat[0]
                    wp.pose.orientation.y = gate_quat[1]
                    wp.pose.orientation.z = gate_quat[2]
                    wp.pose.orientation.w = gate_quat[3]

                    path.poses = [wp]

            elif(self.mode == 'gate'):
                R, t, img, conf = self.gateDetector.predict(img.copy())

                gate_x = rospy.get_param("riseq/gate_x", 0.0)
                gate_y = rospy.get_param("riseq/gate_y", 0.0)
                gate_z = rospy.get_param("riseq/gate_z", 0.0)
                gate_roll = rospy.get_param("riseq/gate_roll", 0.0)
                gate_pitch = rospy.get_param("riseq/gate_pitch", 0.0)
                gate_yaw = rospy.get_param("riseq/gate_yaw", 0.0)

                gate_quat = tf.transformations.quaternion_from_euler(0.0, gate_pitch, gate_roll, axes='rzyx')
            
                # gate waypoint
                wp.pose.position.x = gate_x
                wp.pose.position.y = gate_y
                wp.pose.position.z = gate_z
                wp.pose.orientation.x = gate_quat[0]
                wp.pose.orientation.y = gate_quat[1]
                wp.pose.orientation.z = gate_quat[2]
                wp.pose.orientation.w = gate_quat[3]

            elif(self.mode == 'ladder'):
                bbox, outimg = self.ld.detect(img.copy(), self.max_size)
                if bbox is not None:
                    x,y,w,h = bbox
                    print("Bounding box:",x,y,w,h)

                    ladder_info.depth = 1.5 + np.random.rand()*0.25
                    ladder_info.x = int(x + w/2)
                    ladder_info.y = int(y + h/2)

                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    img = cv2.circle(img, (int(x + w/2),(y + h/2)), 4, (0,255,0), -1)

                else:
                    img = outimg
            
            elif(self.mode == 'irosgate'):
                R, t, R_exp, cnt, mask, cnts = self.ig.detect(img.copy(), self.max_size)
                
                cx_predict = self.cx_kalman.predict()
                cy_predict = self.cy_kalman.predict()
                x_predict = self.x_kalman.predict()[0, 0]
                y_predict = self.y_kalman.predict()[0, 0]
                z_predict = self.z_kalman.predict()[0, 0]

                img = cv2.circle(img, (int(cx_predict[0,0]),int(cy_predict[0,0])), 3, (255,255,0), 3) # estimation

                wp.pose.position.x = x_predict
                wp.pose.position.y = y_predict
                wp.pose.position.z = z_predict

                wp2d.pose.position.x = cx_predict[0,0]
                wp2d.pose.position.y = cy_predict[0,0]

                if R is not None:
                    self.success = self.success + 1.

                    #img = cv2.bitwise_and(img, img, mask = mask)

                    img = cv2.drawContours(img, [cnt[1:]], -1, (255,0,0), 3)
                    img = self.ig.draw_frame(img, (cnt[0][0],cnt[0][1]), R_exp, t)
                    img = cv2.circle(img, (cnt[0][0],cnt[0][1]), 3, (0,255,0), 3 ) # measurement
                    
                    cx_measurement = np.array([[cnt[0][0]*1.0]])
                    cy_measurement = np.array([[cnt[0][1]*1.0]])
                    cx_estimation = self.cx_kalman.correct(cx_measurement)[0,0]
                    cy_estimation = self.cy_kalman.correct(cy_measurement)[0,0]
                    
                    img = cv2.circle(img, (int(cx_estimation),int(cy_estimation)), 3, (0,0,255), 3) # estimation
                    
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis = 0)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T ), axis = 1)
                    gate_quat = tf.transformations.quaternion_from_matrix(R)

                    x_measurement = np.array([[t[2][0]]])
                    y_measurement = np.array([[-t[0][0]]])
                    z_measurement = np.array([[-t[1][0]]])
                    x_estimation = self.x_kalman.correct(x_measurement)[0,0]
                    y_estimation = self.y_kalman.correct(y_measurement)[0,0]
                    z_estimation = self.z_kalman.correct(z_measurement)[0,0]

                    # gate waypoint
                    minval = -50.
                    maxval = 50.
                    #x = np.clip(t[2][0], minval, maxval)
                    #y = np.clip(-t[0][0], minval, maxval)
                    #z = np.clip(np.abs(t[1][0]), minval, maxval)

                    wp.pose.position.x = t[2][0] #x_estimation 
                    wp.pose.position.y = -t[0][0] #y_estimation
                    wp.pose.position.z = -t[1][0] #z_estimation
                    wp.pose.orientation.x = gate_quat[0]
                    wp.pose.orientation.y = gate_quat[1]
                    wp.pose.orientation.z = gate_quat[2]
                    wp.pose.orientation.w = gate_quat[3]

                    wp2d.pose.position.x = cx_estimation
                    wp2d.pose.position.y = cy_estimation

            else:
                
                rospy.loginfo("Monocular Object Detector mode non-existent.")

            path.poses = [wp]          
            self.waypoint_pub.publish(path)
            self.object_centerpoint_pub.publish(wp2d)
            #self.ladder_info_pub.publish(ladder_info)

            img_msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
            self.img_dect_pub.publish(img_msg)
            if(self.enable_recording):
                self.out.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        else:
            # Do nothing
            rospy.loginfo("Monocular mode: {}".format(self.mode))
            pass

        #print("Type: {}, Len: {}, Width: {}, Height: {}, Enc: {}".format(type(image_msg.data), len(image_msg.data), image_msg.width, image_msg.height, image_msg.encoding))
        #img = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)



        #img = Image()
        #img.header.stamp = rospy.Time.now()
        #img.header.frame_id = ""
        #img.height = bbox_img.shape[0]
        #img.width = bbox_img.shape[1]
        #img.encoding = image_msg.encoding
        #img.data = CvBridge().cv2_to_imgmsg(bbox_img, "rgb8")


        #rospy.loginfo(waypoints)

    def camera_params(self, params):
        """
        Receive camera params inside a CameraInfo message
        """
        if not self.camera_info_received:
            self.K = np.array(params.K).reshape(3,3)
            self.D = np.array(params.D)
            self.image_width = params.width
            self.image_height = params.height
            self.camera_info_received = True

    def initKalmanFilters(self):
        self.cx_kalman = cv2.KalmanFilter(2,1,0)
        self.cx_kalman.transitionMatrix = np.array([[1., 0.125], [0., 1.]])
        self.cx_kalman.measurementMatrix = 1. * np.ones((1, 2))
        self.cx_kalman.processNoiseCov = 100* np.eye(2)
        self.cx_kalman.measurementNoiseCov = 1 * np.ones((1, 1))
        self.cx_state = np.array([[self.image_width//2],[0.0]])
        self.cx_kalman.errorCovPost = 1. * np.ones((2, 2))
        self.cx_kalman.statePost = 0.1 * np.random.randn(2, 1)  

        self.cy_kalman = cv2.KalmanFilter(2,1,0)
        self.cy_kalman.transitionMatrix = np.array([[1., 0.125], [0., 1.]])
        self.cy_kalman.measurementMatrix = 1. * np.ones((1, 2))
        self.cy_kalman.processNoiseCov = 100 * np.eye(2)
        self.cy_kalman.measurementNoiseCov = 1 * np.ones((1, 1))
        self.cy_state = np.array([[self.image_height//2],[0.0]])
        self.cy_kalman.errorCovPost = 1. * np.ones((2, 2))
        self.cy_kalman.statePost = 0.1 * np.random.randn(2, 1)

        self.x_kalman = cv2.KalmanFilter(2,1,0)
        self.x_kalman.transitionMatrix = np.array([[1., .125], [0., 1.]])
        self.x_kalman.measurementMatrix = 1. * np.ones((1, 2))
        self.x_kalman.processNoiseCov = 1. * np.eye(2)
        self.x_kalman.measurementNoiseCov = 0.1 * np.ones((1, 1))
        self.x_state = np.array([[self.image_height//2],[0.0]])
        self.x_kalman.errorCovPost = 1. * np.ones((2, 2))
        self.x_kalman.statePost = 0.1 * np.random.randn(2, 1) 

        self.y_kalman = cv2.KalmanFilter(2,1,0)
        self.y_kalman.transitionMatrix = np.array([[1., .125], [0., 1.]])
        self.y_kalman.measurementMatrix = 1. * np.ones((1, 2))
        self.y_kalman.processNoiseCov = 1. * np.eye(2)
        self.y_kalman.measurementNoiseCov = 0.1 * np.ones((1, 1))
        self.y_state = np.array([[self.image_height//2],[0.0]])
        self.y_kalman.errorCovPost = 1. * np.ones((2, 2))
        self.y_kalman.statePost = 0.1 * np.random.randn(2, 1) 

        self.z_kalman = cv2.KalmanFilter(2,1,0)
        self.z_kalman.transitionMatrix = np.array([[1., .125], [0., 1.]])
        self.z_kalman.measurementMatrix = 1. * np.ones((1, 2))
        self.z_kalman.processNoiseCov = 1. * np.eye(2)
        self.z_kalman.measurementNoiseCov = 0.1 * np.ones((1, 1))
        self.z_state = np.array([[self.image_height//2],[0.0]])
        self.z_kalman.errorCovPost = 1. * np.ones((2, 2))
        self.z_kalman.statePost = 0.1 * np.random.randn(2, 1)

def gate_pose_publisher():
    try:
        rospy.init_node("riseq_gate_pose_publisher", anonymous = True)

        monocular_waypoint_publisher = MonoWaypointDetector()

        rospy.loginfo('Gate Pose Publisher Started')
        rospy.spin()
        if monocular_waypoint_publisher.enable_recording:
            print("Saving video recording...")
            monocular_waypoint_publisher.out.release()
        print("Succes rate: {}/{} frames, {:.2f}%".format(monocular_waypoint_publisher.success, monocular_waypoint_publisher.frames, monocular_waypoint_publisher.success/monocular_waypoint_publisher.frames))

        rospy.loginfo('Gate Pose Publisher Terminated')     

    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

if __name__ == '__main__':
    gate_pose_publisher()

