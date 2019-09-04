#!/usr/bin/env python


import rospy
import sys
import numpy as np

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Float64


from helix_trajectory import Helix_Trajectory_Control as htc


class State_Machine():

    def __init__(self):
        self.step = 0

        self.lds = rospy.Subscriber("riseq/sacc/ladder_depth", Float64, self.lds_cb)
        self.ladder_depth = 0.0

        # PX4 related
        self.state = Odometry()
        self.setup_mavros()
        self.command_pose = PoseStamped()
        self.home_pose = PoseStamped()
        self.home_pose_set = False

    def run(self):
        while not rospy.is_shutdown():
            if self.step == 0:
                self.connect_arm_offboard()
                self.step = 1
            elif self.step == 1:
                self.take_off(1.0)
                self.step = 2
            elif self.step == 2:
                self.do_helix_trajectory()
                self.step = 3
            elif(self.step == 3):
                self.return_land_disarm()
                self.step = 4
            elif(self.step == 4):
                break
            else:
                raise
        sys.exit(0)

    def take_off(self, height):
        self.command_pose.pose.position.z = height
        rate = rospy.Rate(10)
        while not (self.state.pose.pose.position.z >= height*0.95 ):
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
        self.helix_controller = htc(vrate = 0.05, radius = 2.0, center = (1,0,0), init=(init_x,init_y,init_z), t_init = rospy.get_time())

        rate = rospy.Rate(30)
        # do helix trajectory for 30 seconds
        print("Doing helix trajectory")
        while( (rospy.get_time() - self.helix_controller.t_init) < 120.):

            # state for x and y position
            xs = np.array([[self.state.pose.pose.position.x],[self.state.twist.twist.linear.x],[0.0]])
            ys = np.array([[self.state.pose.pose.position.y],[self.state.twist.twist.linear.y],[0.0]])
            states = [xs, ys]
            ux, uy, uz = self.helix_controller.compute_command(states, rospy.get_time())

            ct = rospy.get_time() - self.helix_controller.t_init 
            self.helix_controller.set_helix_center((self.ladder_depth*np.cos(0.0),self.ladder_depth*np.sin(1.0)))

            self.command_pose.pose.position.x = ux
            self.command_pose.pose.position.y = uy
            self.command_pose.pose.position.z = uz
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

    def return_land_disarm(self):
        
        # Return home
        print("Return home.")
        rate = rospy.Rate(10)
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 20.0):  # give 20s to go back home
            self.command_pose = self.home_pose
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

    def lds_cb(self, val):
        self.ladder_depth = float(val.data)

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