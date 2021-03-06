#!/usr/bin/env python
"""
author:  Juan Medrano
version: 0.0.1
brief: Publish global and local coordinates for a reference helix trajectory

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
import tf.transformations as tt
import rospy
import sys
import numpy as np

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
from mavros_msgs.msg import GlobalPositionTarget
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Float64
from riseq_sacc.msg import RiseSaccHelix
from sensor_msgs.msg import NavSatFix


from helix_trajectory import Helix_Trajectory_Control as htc
from SISO2 import SISO2_Controller as sc2


class HelixPublisher():

    def __init__(self):
        self.step = 0

        self.lds = rospy.Subscriber("riseq/sacc/ladder_info", RiseSaccHelix, self.lds_cb)
        self.ref_pub_local = rospy.Publisher("riseq/sacc/setpoint_helix_local", PoseStamped, queue_size = 10)
        self.ref_pub_global = rospy.Publisher("riseq/sacc/setpoint_helix_global", GlobalPositionTarget, queue_size = 10)
        self.ladder_depth = None
        self.ladder_height = 30
        self.ladder_safety_margin = 5
        self.ladder_default_global_position = [37.565111 +5*4.579/1000000.0, 126.628503+5*9.812/1000000.0] # lat, lon
        self.width = 1280  # depth image width
        self.height = 720  # depth image height
        self.bbox_x = self.width//2  # assume object is perfectly aligned at start
        self.bbox_y = self.height//2  


        # PX4 related
        self.state = Odometry()
        self.setup_mavros()
        self.command_pose = PoseStamped()
        self.command_pose.pose.orientation.w = 1.0
        self.home_pose = PoseStamped()
        self.home_pose_set = False

        self.global_home = GlobalPositionTarget()
        self.global_state = GlobalPositionTarget()
        self.global_home_pose_set = False

    def run(self):

        self.do_helix_trajectory()
        sys.exit(0)


    def setup_mavros(self):

        self.mavros_state = State()
        self.mavros_state.connected = False

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
     
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_cb)

        self.state.pose.pose.orientation.x = 0.0
        self.state.pose.pose.orientation.y = 0.0
        self.state.pose.pose.orientation.z = 0.0
        self.state.pose.pose.orientation.w = 1.0


    def do_helix_trajectory(self):

        while (not self.home_pose_set) or (not self.global_home_pose_set):
            # do nothing, wait for node initialization to finish
            a = 1

        ladder_position = self.get_ladder_location()
        init_x = self.home_pose.pose.position.x #self.state.pose.pose.position.x
        init_y = self.home_pose.pose.position.y #self.state.pose.pose.position.y
        init_z = self.home_pose.pose.position.z #self.state.pose.pose.position.z
        print("Initial helix position: \n x: {}, y: {}, z: {}\nTime: {}\n".format( init_x, init_y, init_z, rospy.Time.now().to_sec()))
        self.helix_controller = htc(vrate = 0.1, radius = 2.0, center = (1,0,0), init=(init_x,init_y,init_z), t_init = rospy.Time.now().to_sec(), w = 0.2) # init with any parameters
        self.helix_controller.set_helix_center(ladder_position)
        self.yaw_controller = sc2(Kp = 6., Kv = 0.0)
        q = self.state.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        yaw, pitch, roll = tt.euler_from_quaternion(q, axes = 'rzyx')
        self.helix_controller.phase = yaw + np.pi  # yaw angle + 180 degrees because its center perspective
        print("Phase: ",self.helix_controller.phase)

        rate = rospy.Rate(20)
        while(self.state.pose.pose.position.z < (init_z + self.ladder_height + self.ladder_safety_margin)):
            # state for x and y position
            xs = np.array([[self.state.pose.pose.position.x],[self.state.twist.twist.linear.x],[0.0]])
            ys = np.array([[self.state.pose.pose.position.y],[self.state.twist.twist.linear.y],[0.0]])
            states = [xs, ys]

            #ladder_position = self.get_ladder_location()
            #ladder_position = [-29.5,12.5]
            #self.helix_controller.set_helix_center(ladder_position)
            
            ux, uy, uz, ref = self.helix_controller.compute_command(states,  rospy.Time.now().to_sec())
            ux, uy = (ref[0][0][0], ref[1][0][0])
            q, cyaw = self.compute_yaw2(ladder_position)

            self.command_pose.header.stamp = rospy.Time.now()
            self.command_pose.pose.position.x = ux
            self.command_pose.pose.position.y = uy
            self.command_pose.pose.position.z = uz

            self.command_pose.pose.orientation.x = q[0]
            self.command_pose.pose.orientation.y = q[1]
            self.command_pose.pose.orientation.z = q[2]
            self.command_pose.pose.orientation.w = q[3]
            #self.local_pos_pub.publish(self.command_pose)

            # publish reference trajectory in local frame
            refmsg = PoseStamped()
            refmsg.header.stamp = rospy.Time.now()
            refmsg.pose.position.x = ref[0][0][0]
            refmsg.pose.position.y = ref[1][0][0]
            refmsg.pose.position.z = uz
            self.ref_pub_local.publish(refmsg)

            # publish reference trajectory in global frame
            lat,lon = self.get_global_coordinates(ux, uy)
            global_refmsg = GlobalPositionTarget()
            global_refmsg.header.stamp = rospy.Time.now()
            global_refmsg.header.frame_id = self.global_state.header.frame_id
            global_refmsg.latitude = lat
            global_refmsg.longitude = lon
            global_refmsg.yaw = cyaw
            global_refmsg.altitude = uz #(uz - init_z) + self.global_home.altitude
            self.ref_pub_global.publish(global_refmsg)

            rate.sleep()

    def get_global_coordinates(self, x, y):

        # get pose change in local frame
        delta_x = x - self.home_pose.pose.position.x
        delta_y = y - self.home_pose.pose.position.y

        # convert equivalent change in  latitude and longitude
        # see https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        delta_lat = delta_y / 110988.2633
        delta_lon = delta_x / (110988.2633*np.cos(self.global_home.latitude*np.pi/180.0))

        # compute absolute latitude and longitude
        lat = delta_lat + self.global_home.latitude
        lon = delta_lon + self.global_home.longitude
        return lat, lon


    def compute_yaw2(self, ladder_position):
        drone_position = np.array([self.state.pose.pose.position.x,self.state.pose.pose.position.y])
        drone_to_ladder = ladder_position - drone_position
        yaw_ref = np.arctan2(drone_to_ladder[1],drone_to_ladder[0])

        q = self.state.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        yaw, pitch, roll = tt.euler_from_quaternion(q, axes = 'rzyx') 

        state = np.array([[yaw],[0.0],[0.0]])
        ref = np.array([[yaw_ref],[0.0],[0.0]])
        u_yaw = self.yaw_controller.compute_input(state,ref)
        #print("U yaw: {}, Yaw: {}, Yaw_ref: {}".format(u_yaw,yaw, yaw_ref))

        q_ref = tt.quaternion_from_euler(yaw,0,0.0, axes = 'rzyx')

        return q_ref, yaw_ref

    def compute_yaw(self):

        image_center = self.width/2.0

        K =  -2
        pe = K*(self.bbox_x  - image_center)*30/image_center

        q_ref = tt.quaternion_from_euler(pe,0,0.0, axes = 'rzyx')
        return q_ref, pe

    def get_ladder_location(self):

        drone_position = np.array([self.state.pose.pose.position.x,self.state.pose.pose.position.y])
        q = self.state.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        yaw, pitch, roll = tt.euler_from_quaternion(q, axes = 'rzyx')
        if False: #self.ladder_depth is not None: 
            ladder_drone_position = self.ladder_depth*np.array([np.cos(yaw),np.sin(yaw)])
        else:
            # use default ladder position
            delta_y = (self.ladder_default_global_position[0] - self.global_state.latitude)*110988.2633
            delta_x = (self.ladder_default_global_position[1] - self.global_state.longitude)* (110988.2633*np.cos(self.global_home.latitude*np.pi/180.0))
            ladder_drone_position = np.array([delta_x, delta_y])
            print("GPS Delta position: ",delta_y, delta_x)
        ladder_position = ladder_drone_position + drone_position
        print("ladder_position: ",ladder_position) 
        print("drone position:",drone_position)
        return ladder_position


    def position_cb(self, pos):

        if not self.home_pose_set:

            print("\n\n       **********       **********        **********\n"+
                  "                  SET LOCAL HOME POSITION                \n"+
                  " Latitude:  {}\n".format(pos.pose.position.x)+
                  " Longitude: {}\n".format(pos.pose.position.y)+
                  " Altitude: {}\n".format(pos.pose.position.z)+
                  " Time: {}\n".format(rospy.Time.now().to_sec())+
                  "           **********       **********        **********\n\n")

            self.home_pose.pose.position.x = pos.pose.position.x
            self.home_pose.pose.position.y = pos.pose.position.y
            self.home_pose.pose.position.z = pos.pose.position.z
            self.home_pose_set = True

        self.state.pose.pose.position.x = pos.pose.position.x
        self.state.pose.pose.position.y = pos.pose.position.y
        self.state.pose.pose.position.z = pos.pose.position.z

        self.state.pose.pose.orientation.x = pos.pose.orientation.x
        self.state.pose.pose.orientation.y = pos.pose.orientation.y
        self.state.pose.pose.orientation.z = pos.pose.orientation.z
        self.state.pose.pose.orientation.w = pos.pose.orientation.w

    def velocity_cb(self, vel):

        self.state.twist.twist.linear.x = vel.twist.linear.x
        self.state.twist.twist.linear.y = vel.twist.linear.y
        self.state.twist.twist.linear.z = vel.twist.linear.z

        self.state.twist.twist.angular.x = vel.twist.angular.x
        self.state.twist.twist.angular.y = vel.twist.angular.y
        self.state.twist.twist.angular.z = vel.twist.angular.z

    def lds_cb(self, ladder_info):
        
        self.ladder_depth = ladder_info.depth
        self.bbox_x = ladder_info.x
        self.bbox_y = ladder_info.y 
        self.width = ladder_info.width
        self.height = ladder_info.height 

    def global_position_cb(self, gbl_msg):

        if not self.global_home_pose_set:
            print("\n\n       **********       **********        **********\n"+
                  "                  SET GLOBAL HOME POSITION                \n"+
                  " Latitude:  {}\n".format(gbl_msg.latitude)+
                  " Longitude: {}\n".format(gbl_msg.longitude)+
                  " Altitude: {}\n".format(gbl_msg.altitude)+
                  " Time: {}\n".format(rospy.Time.now().to_sec())+
                  "           **********       **********        **********\n\n")
            self.global_home.header.stamp = gbl_msg.header.stamp
            self.global_home.header.frame_id = gbl_msg.header.frame_id
            self.global_home.latitude = gbl_msg.latitude
            self.global_home.longitude = gbl_msg.longitude
            self.global_home.altitude = gbl_msg.altitude
            self.global_home_pose_set = True

        self.global_state.header.stamp = gbl_msg.header.stamp
        self.global_state.header.frame_id = gbl_msg.header.frame_id
        self.global_state.latitude = gbl_msg.latitude
        self.global_state.longitude = gbl_msg.longitude
        self.global_state.altitude = gbl_msg.altitude


if __name__ == '__main__':
    try:
        rospy.init_node('riseq_helix_trajectory_publisher', anonymous = True)

        helix_mission = HelixPublisher()
        helix_mission.run()        

        rospy.loginfo(' Helix Trajectory Publisher Started!')
        rospy.spin()
        rospy.loginfo(' Helix Trajectory Publisher Terminated')    


    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass

