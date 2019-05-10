#!/usr/bin/env python

import rospy
import tf
import numpy as np

from riseq_trajectory.msg import riseq_uav_trajectory
import riseq_tests.df_flat as df_flat 

import trajgen2_helper as trajGen3D


class Trajectory_Generator2():
    def __init__(self):

        # first compute waypoints: the first one is the initial position
        # and orientation, and the rest are the position of the gates

        self.waypoints = self.get_gate_waypoints()
        print("Waypoints: ")
        print(self.waypoints)
        (self.coeff_x, self.coeff_y, self.coeff_z) = trajGen3D.get_MST_coefficients(self.waypoints)

        print("coeff X:")
        print(self.coeff_x)
        print("coeff Y:")
        print(self.coeff_y)
        print("coeff Z:")
        print(self.coeff_z)

        # initialize time for trajectory generator
        self.start_time = rospy.get_time()

        # initialize heading
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        init_quat = [init_pose[3],init_pose[4],init_pose[5],init_pose[6]]
        yaw, pitch, roll = tf.transformations.euler_from_quaternion(init_quat, axes = "rzyx")
        #print("Roll: {}, Pitch: {}, Yaw: {}".format(roll, pitch, yaw))
        trajGen3D.yaw = yaw
        trajGen3D.current_heading = np.array([np.cos(yaw),np.sin(yaw)])
        #print("Yaw: {}, Heading: {}".format(trajGen3D.yaw,trajGen3D.current_heading))



    def compute_reference_traj(self, time):
        vel = 3     #max vel = 3
        trajectory_time = time - self.start_time
        #print("Time traj: {}".format(trajectory_time))
        flatout_trajectory = trajGen3D.generate_trajectory(trajectory_time, vel, self.waypoints, self.coeff_x, self.coeff_y, self.coeff_z)
        ref_trajectory = df_flat.compute_ref(flatout_trajectory)
        return ref_trajectory

    # read initial position and gate positions
    # and return an np.array of these positions
    def get_gate_waypoints(self):

        # Then continue with the gates
        gate_names = rospy.get_param("/uav/gate_names")

        # First waypoint is initial position
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        waypoints = np.zeros((len(gate_names)+1,3))
        waypoints[0][0] = init_pose[0]
        waypoints[0][1] = init_pose[1]
        waypoints[0][2] = init_pose[2]

        for i,name in enumerate(gate_names):
            gate_data = rospy.get_param("/uav/"+name)

            gate_corners = np.array(gate_data['location'])
            #print("Gate {}".format(i))
            #print(gate_corners,type(gate_corners),gate_corners.shape)
            #print("Center:")
            gate_center = np.mean(gate_corners, axis = 0)
            #print(gate_center)

            waypoints[i+1][0] = gate_center[0] 
            waypoints[i+1][1] = gate_center[1]
            waypoints[i+1][2] = gate_center[2]

        waypoints[waypoints.shape[0]-1][1] =  waypoints[waypoints.shape[0]-1][1] - 1

        return waypoints

    def get_vertical_waypoints(self, height):

        # First waypoint is initial position
        init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
        waypoints = np.zeros((2,3))
        waypoints[0][0] = init_pose[0]
        waypoints[0][1] = init_pose[1]
        waypoints[0][2] = init_pose[2]   
        
        # now add a waypoint exactly 1m above the drone 
        waypoints[1][0] = waypoints[0][0]
        waypoints[1][1] = waypoints[0][1]
        waypoints[1][2] = waypoints[0][2] + height

        return waypoints  

def pub_traj():

    # init node
    # rospy.init_node('uav_ref_trajectory_publisher', anonymous = True)
    rospy.init_node('uav_ref_trajectory_input_publisher', anonymous=True)

    # create topic for publishing ref trajectory
    traj_publisher = rospy.Publisher('riseq/tests/uav_simple_trajectory', riseq_uav_trajectory, queue_size=10)


    # wait time for simulator to get ready...
    wait_time = int(rospy.get_param("riseq/trajectory_wait"))
    while( rospy.Time.now().to_sec() < wait_time ):
        if( ( int(rospy.Time.now().to_sec()) % 1) == 0 ):
            rospy.loginfo("Starting Trajectory Generator in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))
    

    # create a trajectory generator
    #traj_gen = Trajectory_Generator()
    traj_gen = Trajectory_Generator2()
    # traj_gen = Trajectory_Generator_Test()

    rospy.sleep(0.1)

    # IMPORTANT WAIT TIME!
    # If this is not here, the "start_time" in the trajectory generator is 
    # initialized to zero (because the node has not started fully) and the
    # time for the trajectory will be degenerated

    # publish at 10Hz
    rate = rospy.Rate(200.0)

    while not rospy.is_shutdown():
        
        try:
            # Compute trajectory at time = now
            time = rospy.get_time()   
            ref_traj = traj_gen.compute_reference_traj(time)

            # create and fill message
            traj = riseq_uav_trajectory()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = ""

            # get all values... we need to convert to np.array()
            # becuase ref_traj contains old, ugly np.matrix
            # objects >:(
            x, y, z = np.array(ref_traj[0]).flatten()
            vx, vy, vz = np.array(ref_traj[1]).flatten()
            ax, ay, az = np.array(ref_traj[10]).flatten()
            jx, jy, jz = np.array(ref_traj[11]).flatten()
            sx, sy, sz = np.array(ref_traj[12]).flatten()

            phi, theta, psi = np.array(ref_traj[2]).flatten()
            p, q, r = np.array(ref_traj[3]).flatten()
            yaw = ref_traj[13].item(0)
            yawdot = ref_traj[14].item(0)
            yawddot = ref_traj[15].item(0)

            uax, uay, uaz = np.array(ref_traj[4]).flatten()
            ubx, uby, ubz = np.array(ref_traj[5]).flatten()
            ucx, ucy, ucz = np.array(ref_traj[6]).flatten()
            Rbw = np.array(ref_traj[9]).flatten().tolist()
            #print("ref_traj[9]: {}".format(ref_traj[9]))

            # Input (T, M) publisher to be used in estimation
            u_1 = np.array(ref_traj[7]).flatten()
            u_xx, u_xy, u_xz = np.array(ref_traj[8]).flatten()


            traj.pose.position.x = x
            traj.pose.position.y = y
            traj.pose.position.z = z

            # Following http://docs.ros.org/jade/api/tf/html/python/transformations.html
            # the axes parameter is such that 
            # r = apply rotation on the "new" frame
            # zyx = this means first a rotation of 'psi' radians around the z axis is performed,
            #       then of 'theta' radians about the new y axis ( because 'r' was specified)
            #       then of 'phi' radians about the new x axis ( becuase 'r' was specified)
            quat = tf.transformations.quaternion_from_euler(psi, theta, phi, axes = 'rzyx')

            traj.pose.orientation.x = quat[0]
            traj.pose.orientation.y = quat[1]
            traj.pose.orientation.z = quat[2]
            traj.pose.orientation.w = quat[3]

            traj.twist.linear.x = vx
            traj.twist.linear.y = vy
            traj.twist.linear.z = vz

            traj.twist.angular.x = p
            traj.twist.angular.y = q
            traj.twist.angular.z = r

            traj.ua.x = uax
            traj.ua.y = uay
            traj.ua.z = uaz

            traj.ub.x = np.linalg.norm(ref_traj[5])  # ubx
            traj.ub.y = uby
            traj.ub.z = ubz

            traj.uc.x = ucx
            traj.uc.y = ucy
            traj.uc.z = ucz

            traj.rot = Rbw

            traj.acc.x = ax 
            traj.acc.y = ay
            traj.acc.z = az

            traj.jerk.x = jx
            traj.jerk.y = jy
            traj.jerk.z = jz

            traj.snap.x = sx
            traj.snap.y = sy
            traj.snap.z = sz

            traj.yaw = yaw
            traj.yawdot = yawdot
            traj.yawddot = yawddot

            # publish message
            traj_publisher.publish(traj)
            rospy.loginfo(traj)
            rate.sleep()

            #traj_gen.trajectory_update(time)

        except Exception:
            rospy.loginfo('People...we have a problem: {}'.format(Exception))
            continue


if __name__ == '__main__':
    try:
        rospy.loginfo("UAV Trajectory Publisher Created")
        pub_traj()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass