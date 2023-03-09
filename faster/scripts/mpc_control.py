#!/usr/bin/env python
from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import rospy
import tf
import math  
from snapstack_msgs.msg import Goal, State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from mpc_tracking_controller import MPCDiffDriveControl 

class MPCControl():
    def __init__(self, delta_t, min_error):
        print("Initializing the MPC controller")
        self.pub_velocity = rospy.Publisher('jackal_velocity_controller/cmd_vel', Twist, queue_size=1, latch=True)
        self.pubState = rospy.Publisher('state', State, queue_size=1, latch=False)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.execute_cb)
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.r = 0.098 # Wheel radius
        self.L = 0.31 # Axle length
        self.D = 0.262 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.odom_pose = None 
        self.error = None
        self.success = False 
        self.min_acceptable_error = min_error 
        self.goal = Goal()
        self.state = State()
        self.target_pose =  np.array([0, 0, 0])
        # self.vehicle_pose = PoseStamped()

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_velocity.publish(msg)
        
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])

        return xwrap[0]
    
    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
        self.odom_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        # self.vehicle_pose = msg 

        self.state = State()
        self.state.pos.x = msg.pose.pose.position.x
        self.state.pos.y = msg.pose.pose.position.y
        self.state.quat = msg.pose.pose.orientation
        self.state.vel = msg.twist.twist.linear
        self.state.w = msg.twist.twist.angular
        self.pubState.publish(self.state)

        # print("states: ", self.state.pos.x, self.state.pos.y)
        # print("target: ", self.target_pose)

    def mpc_planner_init(self):
        self.mpc_solver = MPCDiffDriveControl(self.Ts, 20, 2.2)
        if(self.q is None):
            print("Still robot current pose is not set")
        else:
            self.mpc_solver.init_regulator(self.q, self.target_pose)
        
    def move_one_step(self):
        if(self.mpc_solver.init_reg):
            u, _ = self.mpc_solver.update(self.odom_pose)
            v = u[0]
            w = u[1]
            self.send_vel(v, w)
        elif(self.q is not None):
            self.mpc_solver.init_regulator(self.q, self.target_pose)
        
        if(self.odom_pose is not None):
            self.error = np.linalg.norm(self.odom_pose - self.target_pose)

            self.odom_pose_2D =  np.array([self.odom_pose[0], self.odom_pose[1]])
            self.target_pose_2D =  np.array([self.target_pose[0], self.target_pose[1]])
            self.pos_error = np.linalg.norm(self.odom_pose_2D - self.target_pose_2D)
            
            print("Error: ", self.error, " current state : " , self.odom_pose, "target state: ", self.target_pose)
            print("Position error: ", self.pos_error)

            # err = open('/home/ros/ros_ws/src/faster/faster/scripts/mpc_error.txt', 'a')
            # err.write(str(self.error)+"\n")
            # err.close()

            # tar = open('/home/ros/ros_ws/src/faster/faster/scripts/target_pose.txt', 'a')
            # tar.write(str(self.target_pose)+"\n")
            # tar.close()

            # od = open('/home/ros/ros_ws/src/faster/faster/scripts/odom_pose.txt', 'a')
            # od.write(str(self.odom_pose)+"\n")
            # od.close()

            if(self.error < self.min_acceptable_error):
                self.success = True 

                # self.init_reg = False 
                # self.send_vel(0, 0)
                # self.end_controller = True
        
    def goalCB(self, goal):
        self.goal = goal
        yaw = math.atan2(self.goal.p.y - self.odom_pose[1], self.goal.p.x - self.odom_pose[0])
        self.target_pose =  np.array([self.goal.p.x, self.goal.p.y, yaw])

    def execute_cb(self, goal):
        # rospy.loginfo('%s: Executing, moving to a target location %i,%i with current error %i' % 
        #             ( target_pose[0], target_pose[1])
        self.mpc_planner_init()
        self.move_one_step()

        if (self.success == True):
            self.success = False
            # self.init_reg = False
            


def startNode():
    controller = MPCControl(delta_t=0.05, min_error=0.2)

    rospy.Subscriber("goal", Goal, controller.goalCB, queue_size=1)
    rospy.Subscriber("ground_truth/state", Odometry, controller.set_pose, queue_size=1) #jackal_velocity_controller/odom   # odometry/local_filtered

    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('mpc_control')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
        else:
            print ("Starting node for: "+ ns)
            startNode()
    except rospy.ROSInterruptException:
        pass