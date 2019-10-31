#!/usr/bin/env python

import tf.transformations as tt
import rospy
import sys
import numpy as np

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State 
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
from riseq_perception.irosgate_searcher import IROSGateSearcher


from statemachine import StateMachine
from statemachine import State as _State_

class IROS_StateMachine(StateMachine):

    Landed = _State_('Land', initial=True)
    Hovering = _State_('Hovering')
    Search = _State_('Search')
    Fly = _State_('Flying to Gate')
    Turn_Advance = _State_('Turn Around and advance')

    take_off = Landed.to(Hovering)
    gate_not_found = Hovering.to(Search)
    gate_found = Search.to(Fly)
    gate_pass = Fly.to(Turn_Advance)
    repeat = Turn_Advance.to(Hovering)

    def on_take_off(self):
        print("Taking off...")

    def on_gate_not_found(self):
        print("Gate Not Found. Searching...")

    def on_gate_found(self):
        print("Gate Found! Flying to Gate...")

    def on_gate_pass(self):
        print("Gate Passed!")

    def on_repeat(self):
        print("Going for next Gate")

class IROS_Coordinator():

    def __init__(self):

        self.home_pose_set = False
        self.setup_iros()
        self.setup_mavros()
 
    def main_process(self):
        while not rospy.is_shutdown():

            if self.machine.is_Landed:
                self.connect_arm_offboard()
                self.take_off(self.hover_height)
                self.machine.take_off()

            elif self.machine.is_Hovering:
                self.machine.gate_not_found()

            elif self.machine.is_Search:
                if self.gate_found():
                    self.gate_searcher.searching = False
                    if self.gate_classified():
                        self.fly_global_coordinates = True
                        self.fly_local_coordinates = False
                    else:
                        self.fly_global_coordinates = False
                        self.fly_local_coordinates = True
                    self.machine.gate_found()
                else:
                    # keep searching
                    if self.gate_searcher.searching: 
                        cont, yaw = self.gate_searcher.search_gate()
                        if cont: # continue searching?
                            self.yaw_rotate(yaw*180.0/np.pi)
                        else:
                            print("Gate Not Found")
                            self.return_land_disarm()
                    else:
                        # 
                        init_yaw, pitch, roll = tt.euler_from_quaternion(self.orientation.tolist(), axes = 'rzyx')
                        self.gate_searcher.init_yaw = init_yaw
                        self.gate_searcher.yaw = init_yaw
                        self.gate_searcher.searching = True
            
            elif self.machine.is_Fly:
                success = self.Fly_To_Gate()
                if success:
                    self.machine.gate_pass()

            elif self.machine.is_Turn_Advance:
                self.Turn_Advance()
                self.machine.repeat()

            else:
                print("FATAL ERROR:\nCurrent machine state is not supported: {}".format(self.machine.current_state))
                self.return_land_disarm()

        sys.exit(0)

    def Fly_To_Gate(self):
        # align drone with gate in YZ plane
        gate_position = self.average_gate_position(1) + self.drone_camera_offset_vector
        Rbw = self.get_body_to_world_matrix()
        goal_position_yz = self.position + np.dot(Rbw, gate_position.reshape(3,1)).reshape(3)
        goal_position_yz[0] = self.position[0]  # only move in YZ plane, so make X coordinate the same as current
        self.go_position(goal_position_yz)


        if self.fly_global_coordinates:
            gate_position = self.get_gate_global_position()
            if gate_position is not None:
                print("Global Coordinates Flight to: {}".format(self.gate_type))
                goal_position = gate_position + self.vo_drift   # drift compensation only in XY plane
                
                gate_position = self.average_gate_position(1) + self.drone_camera_offset_vector
                Rbw = self.get_body_to_world_matrix()
                gate_position = self.position + np.dot(Rbw, gate_position.reshape(3,1)).reshape(3)

                goal_position[2] = gate_position[2] # 

                # add correction offset to ensure gate pass
                if(goal_position[0]<0):
                    goal_position[0] = goal_position[0] - self.gate_correction_offset[0]
                else:
                    goal_position[0] = goal_position[0] + self.gate_correction_offset[0]

                self.go_position(goal_position)
            else:
                # try to fly in global coordinates
                self.fly_global_coordinates = False
                self.fly_local_coordinates = True
                print(" Global Coordinates Flight Aborted. Trying Local coordinates...")
                return False
        else:
            # fly in local coordinates
            print("Local Coordinates Flight...")
            gate_position_drone_frame = self.average_gate_position(1) + self.drone_camera_offset_vector + self.gate_correction_offset
            Rbw = self.get_body_to_world_matrix()
            gate_position_local_frame = self.position + np.dot(Rbw, gate_position_drone_frame.reshape(3,1)).reshape(3)
            self.go_position(gate_position_local_frame)  

        return True      

    def get_gate_global_position(self):
        if self.gate_type == 'left':
            return self.gate_left   
        elif self.gate_type == 'right':
            return self.gate_right
        elif self.gate_type == 'vertical':
            return self.gate_up
        else:
            print("Gate Type : {} is NOT correct for global coordinate flight".format(self.gate_type))
            return None

    def Turn_Advance(self):

        if self.fly_global_coordinates:
            if self.gate_type == 'vertical' or self.gate_type == 'left':
                sideways_vector = np.array([[0],[self.one_block],[0]])
            elif self.gate_type == 'right':
                sideways_vector = np.array([[0],[self.two_block],[0]])
            else:
                # if for some reason the gate type is lost
                sideways_vector = np.array([[0],[self.two_block],[0]])
        else:
            # Move in local coordinates without knowledge of which gate pass just passed
            # Move two "blocks"
            sideways_vector = np.array([[0],[self.two_block],[0]])

        Rbw = self.get_body_to_world_matrix()
        goal_pose = self.position + np.dot(Rbw, sideways_vector).reshape(3)
        # Adjust to hover height
        goal_pose[2] = self.hover_height        
        self.go_position(goal_pose)

        # Rotate
        self.yaw_rotate(180)   

        # Advance forward
        Rbw = self.get_body_to_world_matrix()
        forward_vector = np.array([[self.advance_distance],[0],[0]])
        goal_pose = self.position + np.dot(Rbw, forward_vector).reshape(3)
        self.go_position(goal_pose)
        return True

    def get_body_to_world_matrix(self):
        Rwb = tt.quaternion_matrix(self.orientation.tolist())
        Rbw = Rwb[0:3,0:3].T
        return Rbw

    def average_gate_position(self, samples):

        avg_position = np.zeros(3)
        i = self.gate_msgs_count + samples
        o = self.gate_msgs_count
        while self.gate_msgs_count < i:
            if o == self.gate_msgs_count:
                avg_position = avg_position + self.gate_position
                o = o + 1
            else:
                pass 
        avg_position = avg_position/samples
        return self.gate_position

    def gate_classified(self):
        if self.gate_type == 'unknown':
            return False
        else:
            return True

    def gate_found(self):
        
        if self.time_of_last_gate_position is None:
            # gate still not found
            return False
        elif ( rospy.Time.now().secs - self.time_of_last_gate_position) > self.gate_detection_wait_time :
            # wait time passed... conclude that gate was not detected
            return False
        else:  
            return True

    def setup_mavros(self):

        self.state_msg = PoseStamped()
        self.state_msg.pose.orientation.w = 1.0    # avoid weird values if w = 0 when starting
        self.command_pose_msg = PoseStamped()
        self.command_pose_msg.pose.orientation.w = 1.0

        self.mavros_state = State()
        self.mavros_state.connected = False

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
     
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.mavros_state_sub = rospy.Subscriber('mavros/state', State, self.mavros_status_cb)        
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        #self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)

    def setup_iros(self):

        self.machine = IROS_StateMachine()
        self.gate_searcher = IROSGateSearcher(initial_yaw = 0)

        self.vo_drift = np.zeros(3)
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)
        self.command_pose = np.zeros(3)
        self.home_pose = np.zeros(3)
        self.gate_position = np.zeros(3)
        self.time_of_last_gate_position = None
        self.fly_global_coordinates = False
        self.fly_local_coordinates = True    # fly in local coordinates by default
        self.gate_msgs_count = 0
        self.gate_type = "unknown"

        self.hover_height = rospy.get_param("/drone/hover_height", 1.5)
        self.gate_detection_wait_time = rospy.get_param("/perception/gate_detection_wait_time", 1.0)
        self.position_error_threshold = rospy.get_param("/drone/position_error_threshold", 0.2)
        self.drone_camera_offset_vector = np.array(rospy.get_param("/drone/drone_camera_offset_vector",[0.14,0,0]))
        self.gate_correction_offset = np.array(rospy.get_param("/drone/gate_correction_offset", [1.0,0,0]))
        self.one_block = rospy.get_param("/drone/one_block", 1.4)
        self.two_block = rospy.get_param("/drone/two_block", 2.0)
        self.advance_distance = rospy.get_param("/drone/advance_distance", 5)
        self.gate_up = rospy.get_param('/gates/gate_vertical/up', [6, 0.0, 2.7])
        self.gate_left = rospy.get_param('/gates/gate_horizontal/left', [-4, 0.0, 2])
        self.gate_right = rospy.get_param('/gates/gate_horizontal/right', [-4, 1.4, 2])

        rospy.Subscriber("/riseq/drone/vo_drift", PoseStamped, self.vo_drift_cb)
        rospy.Subscriber("/riseq/gate/observing", String, self.gate_classification_cb)
        rospy.Subscriber("/riseq/perception/uav_mono_waypoint", PoseStamped, self.gate_waypoint_cb)

    def vo_drift_cb(self, drift):
        self.vo_drift[0] = drift.pose.position.x
        self.vo_drift[1] = drift.pose.position.y
    
    def gate_classification_cb(self,msg):
        self.gate_type = msg.data


    def connect_arm_offboard(self):
        self.wait_mavros_connection()
        self.last_mavros_request = rospy.Time.now()
        self.enable_sim = rospy.get_param('/drone/auto_arm_offboard', False)
        if(self.enable_sim):
            self.send_setpoints()
            self.status_timer = rospy.Timer(rospy.Duration(0.5), self.mavros_arm_offboard_cb)

        armed = self.mavros_state.armed
        mode = self.mavros_state.mode
        rate = rospy.Rate(20.0)
        while( (not armed ) or (mode != 'OFFBOARD')):
            armed = self.mavros_state.armed
            mode = self.mavros_state.mode
            rate.sleep()
        print("ARMED, OFFBOARD MODE SET")

    def mavros_arm_offboard_cb(self, timer):

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

    def wait_mavros_connection(self):
        i = 0
        rate = rospy.Rate(20)
        while not self.mavros_state.connected:
            print (" >> {} NOT CONNECTED TO MAVROS <<".format(i))
            rate.sleep()
            i  = i + 1
        print(">> {} CONNECTED TO MAVROS! <<".format(i))
    
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

    def mavros_status_cb(self, state_msg):
        self.mavros_state = state_msg 

    def position_cb(self, pos):

        if not self.home_pose_set:
            self.home_pose[0] = pos.pose.position.x
            self.home_pose[1] = pos.pose.position.y
            self.home_pose[2] = pos.pose.position.z
            self.home_pose_set = True

        self.position[0] = pos.pose.position.x
        self.position[1] = pos.pose.position.y
        self.position[2] = pos.pose.position.z

        self.orientation[0] = pos.pose.orientation.x
        self.orientation[1] = pos.pose.orientation.y
        self.orientation[2] = pos.pose.orientation.z
        self.orientation[3] = pos.pose.orientation.w

    def velocity_cb(self, vel):

        self.position.twist.twist.linear.x = vel.twist.linear.x
        self.position.twist.twist.linear.y = vel.twist.linear.y
        self.position.twist.twist.linear.z = vel.twist.linear.z

        self.position.twist.twist.angular.x = vel.twist.angular.x
        self.position.twist.twist.angular.y = vel.twist.angular.y
        self.position.twist.twist.angular.z = vel.twist.angular.z

    def take_off(self, height):
        self.command_pose[2] = height
        rate = rospy.Rate(20)
        while (self.position[2] < height*0.95 ):
            self.publish_command(self.command_pose)
            rate.sleep()

    def set_yaw_rotation(self, angle):
        yaw = angle*np.pi/180.
        print("Yawing: {} degrees".format(angle))
        start = rospy.get_time()
        q90 = tt.quaternion_from_euler(yaw,0,0.0, axes = 'rzyx')

        self.command_pose_msg.pose.orientation.x = q90[0]
        self.command_pose_msg.pose.orientation.y = q90[1]
        self.command_pose_msg.pose.orientation.z = q90[2]
        self.command_pose_msg.pose.orientation.w = q90[3]



        rate = rospy.Rate(20)
        while( (rospy.get_time() - start) < 1):
            self.command_pose_msg.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose_msg)
            rate.sleep()  

    def yaw_rotate(self, angle):
        """
        Rotate around yaw axis by 'angle' degrees from current orientation
        """
        yaw = angle*np.pi/180.
        print("Yawing: {:.2f} degrees".format(angle))
        start = rospy.get_time()

        Rdes = tt.euler_matrix(yaw,0.0,0.0, axes = 'rzyx')  # desired rotation as matrix
        R = tt.quaternion_matrix(self.orientation.tolist()) # current rotation as matrix
        Rcmd = np.dot(Rdes,R)                               # total rotation as matrix
        qcmd = tt.quaternion_from_matrix(Rcmd)              # total rotaion as quaternion

        self.command_pose_msg.pose.orientation.x = qcmd[0]
        self.command_pose_msg.pose.orientation.y = qcmd[1]
        self.command_pose_msg.pose.orientation.z = qcmd[2]
        self.command_pose_msg.pose.orientation.w = qcmd[3]

        rate = rospy.Rate(20)
        while( (rospy.get_time() - start) < 5.0):
            self.command_pose_msg.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose_msg)
            rate.sleep()

    def go_position(self,position):

        self.command_pose[0] = position[0]     
        self.command_pose[1] = position[1]
        self.command_pose[2] = position[2]

        rate = rospy.Rate(20)
        while (self.position_error(self.command_pose, self.position) >= self.position_error_threshold ):
            self.publish_command(self.command_pose)
            rate.sleep()

        return True

    def position_error(self, v1, v2):
        return np.linalg.norm(v1 - v2)
        
    def return_land_disarm(self):
        
        # Return home
        print("Returning home.")
        rate = rospy.Rate(20)
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 10.0):  # give 20s to go back home
            self.command_pose = self.home_pose
            self.publish_command(self.command_pose)
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

    def publish_command(self, command):

        self.command_pose_msg.header.stamp = rospy.Time.now()
        self.command_pose_msg.pose.position.x = command[0]
        self.command_pose_msg.pose.position.y = command[1]
        self.command_pose_msg.pose.position.z = command[2]
        self.local_pos_pub.publish(self.command_pose_msg)

    def gate_waypoint_cb(self, msg):
        self.gate_position[0] = msg.pose.position.x
        self.gate_position[1] = msg.pose.position.y
        self.gate_position[2] = msg.pose.position.z
        self.time_of_last_gate_position = msg.header.stamp.secs
        self.gate_msgs_count = self.gate_msgs_count + 1

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_iros_state_machine', anonymous = True)

        iros_mission = IROS_Coordinator()
        iros_mission.main_process()        

        rospy.loginfo('IROS RISEQ STATE MACHINE STARTED')
        rospy.spin()
        rospy.loginfo('IROS RISEQ STATE MACHINE TERMINATED')    


    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass
