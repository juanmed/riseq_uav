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
        # 
        self.hover_height = rospy.get_param("/drone/hover_height", 1.5)

        self.machine = IROS_StateMachine()

        # PX4 related
        self.state = PoseStamped()
        self.command_pose = PoseStamped()
        self.home_pose = PoseStamped()
        self.home_pose_set = False
        self.setup_mavros()

    def main_process(self):
        while not rospy.is_shutdown():

            if self.machine.is_Landed:
                self.connect_arm_offboard()
                self.take_off(1.0)
                self.machine.take_off()

            elif self.machine.is_Hovering:
                self.machine.gate_not_found()

            elif self.machine.is_Search:
                if self.gate_found():
                    if self.gate_classified():
                        self.fly_global_coordinates()
                    else:
                        self.fly_local_coordinates()
                    self.machine.gate_found()
                else:
                    # keep searching
                    pass
            
            elif self.machine.is_Fly:
                if self.fly_global_coordinates():
                    # do drift correction and fly to gate
                    self.go_position([1,0,self.hover_height])
                else:
                    # fly in local coordinates
                    self.go_position([1,0,self.hover_height])
                self.machine.gate_pass()

            elif self.machine.is_Turn_Advance:
                self.go_position([2,2,self.hover_height])
                self.yaw_rotate(90)
                self.machine.repeat()
            else:
                print("Current machine state is not supported: {}".format(self.machine.current_state))


        sys.exit(0)

    def fly_global_coordinates(self):
        return True

    def fly_local_coordinates(self):
        return 0

    def gate_classified(self):
        return False

    def gate_found(self):
        return True

    def vo_drift_cb(self, drift):
        return True

    def setup_mavros(self):

        self.mavros_state = State()
        self.mavros_state.connected = False

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
     
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.mavros_state_sub = rospy.Subscriber('mavros/state', State, self.mavros_status_cb)        
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        #self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)

        self.state.pose.orientation.x = 0.0
        self.state.pose.orientation.y = 0.0
        self.state.pose.orientation.z = 0.0
        self.state.pose.orientation.w = 1.0

        self.command_pose.pose.orientation.w = 1.0

    def connect_arm_offboard(self):
        self.wait_mavros_connection()
        self.last_mavros_request = rospy.Time.now()
        self.enable_sim = rospy.get_param('/riseq/enable_sim', False)
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
            self.home_pose.pose.position.x = pos.pose.position.x
            self.home_pose.pose.position.y = pos.pose.position.y
            self.home_pose.pose.position.z = pos.pose.position.z
            self.home_pose_set = True

        self.state.pose.position.x = pos.pose.position.x
        self.state.pose.position.y = pos.pose.position.y
        self.state.pose.position.z = pos.pose.position.z

        self.state.pose.orientation.x = pos.pose.orientation.x
        self.state.pose.orientation.y = pos.pose.orientation.y
        self.state.pose.orientation.z = pos.pose.orientation.z
        self.state.pose.orientation.w = pos.pose.orientation.w

    def velocity_cb(self, vel):

        self.state.twist.twist.linear.x = vel.twist.linear.x
        self.state.twist.twist.linear.y = vel.twist.linear.y
        self.state.twist.twist.linear.z = vel.twist.linear.z

        self.state.twist.twist.angular.x = vel.twist.angular.x
        self.state.twist.twist.angular.y = vel.twist.angular.y
        self.state.twist.twist.angular.z = vel.twist.angular.z

    def take_off(self, height):
        self.command_pose.pose.position.z = height
        rate = rospy.Rate(20)
        while not (self.state.pose.position.z >= height*0.95 ):
            self.command_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

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

    def go_position(self,position):

        self.command_pose.pose.position.x = position[0]     
        self.command_pose.pose.position.y = position[1]
        self.command_pose.pose.position.z = position[2]

        rate = rospy.Rate(20)
        while (self.position_error(self.command_pose, self.state) >= 0.2 ):
            self.command_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

        return True

    def position_error(self, v1, v2):
        v1 = np.array([v1.pose.position.x, v1.pose.position.y, v1.pose.position.z])
        v2 = np.array([v2.pose.position.x, v2.pose.position.y, v2.pose.position.z])
        e = np.linalg.norm(v1 - v2)
        return e

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