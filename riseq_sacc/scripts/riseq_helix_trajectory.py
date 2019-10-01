#!/usr/bin/env python

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


class State_Machine():

    def __init__(self):
        self.step = 0

        self.lds = rospy.Subscriber("riseq/sacc/ladder_info", RiseSaccHelix, self.lds_cb)
        self.ref_pub_local = rospy.Publisher("riseq/sacc/setpoint_helix_local", PoseStamped, queue_size = 10)
        self.ref_pub_global = rospy.Publisher("riseq/sacc/setpoint_helix_global", GlobalPositionTarget, queue_size = 10)
        self.ladder_depth = 0.0
        self.ladder_height = 30

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
        while not rospy.is_shutdown():
            if self.step == 0:
                self.connect_arm_offboard()
                self.step = 1
            elif self.step == 1:
                self.take_off(1.0)
                self.step = 2
            elif self.step == 2:
                self.yaw_rotate(130)
                self.step = 3
            elif self.step == 3:
                self.do_helix_trajectory()
                self.step = 4
            elif(self.step == 4):
                self.return_land_disarm()
                self.step = 5
            elif(self.step == 5):
                break
            else:
                raise
        sys.exit(0)

    def yaw_rotate(self, angle):
        yaw = angle*np.pi/180.
        print("YAW90")
        start = rospy.get_time()
        q90 = tt.quaternion_from_euler(yaw,0,0.0, axes = 'rzyx')
        self.command_pose.pose.orientation.x = q90[0]
        self.command_pose.pose.orientation.y = q90[1]
        self.command_pose.pose.orientation.z = q90[2]
        self.command_pose.pose.orientation.w = q90[3]
        rate = rospy.Rate(20)
        while( (rospy.get_time() - start) < 5):
            self.command_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()            

    def take_off(self, height):
        self.command_pose.pose.position.z = height
        rate = rospy.Rate(20)
        while not (self.state.pose.pose.position.z >= height*0.95 ):
            self.command_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

    def setup_mavros(self):

        self.mavros_state = State()
        self.mavros_state.connected = False

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
     
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.mavros_state_sub = rospy.Subscriber('mavros/state', State, self.mavros_state_cb)        
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)
        self.global_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_cb)

        self.state.pose.pose.orientation.x = 0.0
        self.state.pose.pose.orientation.y = 0.0
        self.state.pose.pose.orientation.z = 0.0
        self.state.pose.pose.orientation.w = 1.0

    def mavros_state_cb(self, state_msg):
        self.mavros_state = state_msg 

    def wait_mavros_connection(self):
        i = 0
        rate = rospy.Rate(20)
        while not self.mavros_state.connected:
            print (" >> {} NOT CONNECTED TO MAVROS <<".format(i))
            rate.sleep()
            i  = i + 1
        print(">> {} CONNECTED TO MAVROS! <<".format(i))

    def connect_arm_offboard(self):
        self.wait_mavros_connection()
        self.last_mavros_request = rospy.Time.now()
        self.enable_sim = rospy.get_param('/riseq/enable_sim', False)
        if(self.enable_sim):
            self.send_setpoints()
            self.status_timer = rospy.Timer(rospy.Duration(0.5), self.mavros_status_cb)

        armed = self.mavros_state.armed
        mode = self.mavros_state.mode
        rate = rospy.Rate(20.0)
        while( (not armed ) or (mode != 'OFFBOARD')):
            armed = self.mavros_state.armed
            mode = self.mavros_state.mode
            rate.sleep()
        print("ARMED, OFFBOARD MODE SET")

    def send_setpoints(self):
        """
        Publish 100 position setpoints before arming.
        This is required for successfull arming
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        rate = rospy.Rate(20.0)
        for i in range(100):
            self.local_pos_pub.publish(pose)
            rate.sleep()

    def mavros_status_cb(self, timer):

        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"
        arm_cmd = CommandBool()
        arm_cmd.value = True

        if(self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_mavros_request > rospy.Duration(5.0))):
            resp1 = self.set_mode_client(0,offb_set_mode.custom_mode)
            if resp1.mode_sent:
                #rospy.loginfo("Requested OFFBOARD")
                pass
            self.last_mavros_request = rospy.Time.now()
        elif (not self.mavros_state.armed and (rospy.Time.now() - self.last_mavros_request > rospy.Duration(5.0))):
            arm_client_1 = self.arming_client(arm_cmd.value)
            if arm_client_1.success:
                #rospy.loginfo("Requested Vehicle ARM")
                pass
            self.last_mavros_request = rospy.Time.now() 
        
        armed = self.mavros_state.armed
        mode = self.mavros_state.mode
        rospy.loginfo("Vehicle armed: {}".format(armed))
        rospy.loginfo("Vehicle mode: {}".format(mode))

        if( (not armed ) or (mode != 'OFFBOARD')):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0            
            self.local_pos_pub.publish(pose)
        else:
            self.status_timer.shutdown()

    def do_helix_trajectory(self):

        init_x = self.state.pose.pose.position.x
        init_y = self.state.pose.pose.position.y
        init_z = self.state.pose.pose.position.z
        self.helix_controller = htc(vrate = 0.05, radius = 1.0, center = (1,0,0), init=(init_x,init_y,init_z), t_init = rospy.get_time(), w = 0.5)
        self.yaw_controller = sc2(Kp = 6., Kv = 0.0)
        q = self.state.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        yaw, pitch, roll = tt.euler_from_quaternion(q, axes = 'rzyx')
        self.helix_controller.phase = yaw + np.pi  # yaw angle + 180 degrees because its center perspective
        print("Phase: ",self.helix_controller.phase)

        rate = rospy.Rate(20)
        # do helix trajectory for 30 seconds
        print("Doing helix trajectory")
        #while( (rospy.get_time() - self.helix_controller.t_init) < 120.):
        while(self.state.pose.pose.position.z < (self.ladder_height + 2)):
            # state for x and y position
            xs = np.array([[self.state.pose.pose.position.x],[self.state.twist.twist.linear.x],[0.0]])
            ys = np.array([[self.state.pose.pose.position.y],[self.state.twist.twist.linear.y],[0.0]])
            states = [xs, ys]

            ladder_position = self.get_ladder_location()
            ladder_position = [0.0,1.0]
            self.helix_controller.set_helix_center(ladder_position)
            
            ux, uy, uz, ref = self.helix_controller.compute_command(states, rospy.get_time())
            q = self.compute_yaw()

            self.command_pose.header.stamp = rospy.Time.now()
            self.command_pose.pose.position.x = ux
            self.command_pose.pose.position.y = uy
            self.command_pose.pose.position.z = uz

            self.command_pose.pose.orientation.x = q[0]
            self.command_pose.pose.orientation.y = q[1]
            self.command_pose.pose.orientation.z = q[2]
            self.command_pose.pose.orientation.w = q[3]
            self.local_pos_pub.publish(self.command_pose)

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
            global_refmsg.altitude = uz
            self.ref_pub_global.publish(global_refmsg)

            rate.sleep()

    def get_global_coordinates(self, x, y):

        # get pose change in local frame
        delta_x = x - self.home_pose.pose.position.x
        delta_y = y - self.home_pose.pose.position.y

        # convert equivalent change in  latitude and longitude
        # see https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        delta_lat = delta_y / 111111.0
        delta_lon = delta_x / (111111.0*np.cos(self.global_home.latitude*np.pi/180.0))

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

        return q_ref

    def compute_yaw(self):

        image_center = self.width/2.0

        K = 2.0
        pe = -K*(self.bbox_x  - image_center)/image_center

        q_ref = tt.quaternion_from_euler(pe,0,0.0, axes = 'rzyx')
        return q_ref

    def get_ladder_location(self):

        drone_position = np.array([self.state.pose.pose.position.x,self.state.pose.pose.position.y])
        q = self.state.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        yaw, pitch, roll = tt.euler_from_quaternion(q, axes = 'rzyx')
        ladder_drone_position = self.ladder_depth*np.array([np.cos(yaw),np.sin(yaw)])
        ladder_position = ladder_drone_position #+ drone_position 
        return ladder_position

    def return_land_disarm(self):
        
        # Return home
        print("Return home.")
        rate = rospy.Rate(20)
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 20.0):  # give 20s to go back home
            self.command_pose = self.home_pose
            self.command_pose.header.stamp = rospy.Time.now()            
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

        # Land
        print("Landing.")
        self.landing_client(0.0, 0.0, 0.0, 0.0, 0.0)
        rospy.sleep(5)        

        # disarm
        print("Disarm.")
        arm_cmd = CommandBool()
        arm_cmd.value = False
        arm_client_1 = self.arming_client(arm_cmd.value)

    def position_cb(self, pos):

        if not self.home_pose_set:
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
        rospy.init_node('riseq_helix_mission', anonymous = True)

        helix_mission = State_Machine()
        helix_mission.run()        

        rospy.loginfo(' Helix Mission Started!')
        rospy.spin()
        rospy.loginfo(' HHelix Mission Terminated')    


    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass

