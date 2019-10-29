import rospy
import sys
import numpy as numpy

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State 

from statemachine import StateMachine, State	

class IROS_StateMachine(StateMachine):

    Landed = State('Land', initial=True)
    Hovering = State('Hovering')
    Search = State('Search')
    Fly = State('Flying to Gate')
    Turn_Advance = State('Turn Around and advance')

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



		self.machine = IROS_StateMachine()

        # PX4 related
        self.state = Odometry()
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

	    	if self.machine.is_Hovering:
	    		if self.gate_found:
	    			if self.gate_classified:
	    				self.fly_global()
	    			else:

	    		else:





        sys.exit(0)
    def vo_drift_cb(self, drift):

    def setup_mavros(self):

        self.mavros_state = State()
        self.mavros_state.connected = False

        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
     
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.mavros_state_sub = rospy.Subscriber('mavros/state', State, self.mavros_status_cb)        
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)

        self.state.pose.pose.orientation.x = 0.0
        self.state.pose.pose.orientation.y = 0.0
        self.state.pose.pose.orientation.z = 0.0
        self.state.pose.pose.orientation.w = 1.0

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

    def take_off(self, height):
        self.command_pose.pose.position.z = height
        rate = rospy.Rate(20)
        while not (self.state.pose.pose.position.z >= height*0.95 ):
            self.command_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.command_pose)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_iros_state_machine', anonymous = True)

        helix_mission = State_Machine()
        helix_mission.run()        

        rospy.loginfo('IROS RISEQ STATE MACHINE STARTED')
        rospy.spin()
        rospy.loginfo('IROS RISEQ STATE MACHINE TERMINATED')    


    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass