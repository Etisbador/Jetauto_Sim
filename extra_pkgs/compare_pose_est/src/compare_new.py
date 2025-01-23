#!/usr/bin/env python3

import rospy
import time
import subprocess
import rosbag
import matplotlib.pyplot as plt
import os
import sys
import rospkg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist
from math import sqrt
import time
from std_msgs.msg import Float32MultiArray, Header
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


x_gazebo = []
y_gazebo = []
x_sim = []
y_sim = []
x_real = []
y_real = []

class Compare_Pose_Est():
    def __init__(self):
        rospy.init_node('compare_pose_est', anonymous=False)   
        
        rospy.Subscriber("/odom", Odometry, self.odom_gazebo)
        rospy.Subscriber("/jetauto_odom_sim", Odometry, self.odom_sim)
        rospy.Subscriber("/jetauto_odom", Odometry, self.odom_real)
        
        #self.control_publisher = rospy.Publisher("wheel_vel", Float32MultiArray, queue_size=10)  
        self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)  
               
        
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.path_pub = rospy.Publisher('/path_ref', Path, queue_size=10)
        self.path_sim_pub = rospy.Publisher('/path_sim', Path, queue_size=10)
              
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
        
        # Iniciar la posición del roboto en gazebo
        
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        
        rospy.sleep(1.5)
       
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            model_state = ModelState()
            model_state.model_name = 'jetauto'
            model_state.pose = Pose()
            model_state.pose.position.x = 0.0
            model_state.pose.position.y = 0.0
            model_state.pose.position.z = 0.0
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = 0.0
            model_state.pose.orientation.w = 1.0

            model_state.twist = Twist()
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0

            model_state.reference_frame = 'world'

            resp = set_state(model_state)
            rospy.sleep(1.0)
            
            if resp.success:
                rospy.loginfo("Model state set successfully")
            else:
                rospy.logerr("Failed to set model state: %s", resp.status_message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
                   
    
        #Definir Path real RVIZ
        self.path = Path()
        self.path.header = Header()
        self.path.header.stamp = self.sim_time
        self.path.header.frame_id = 'odom'  # Frame of reference
        
        #Definir Path simulacion RVIZ
        self.path_sim = Path()
        self.path_sim.header = Header()
        self.path_sim.header.stamp = self.sim_time
        self.path_sim.header.frame_id = 'odom'  # Frame of reference    
        
        self.posx_gazebo = 0.0
        self.posy_gazebo = 0.0
        
        self.posx_sim = 0.0
        self.posy_sim = 0.0
        
        self.posx_real = 0.0
        self.posy_real = 0.0
##BAG
        bag_path = "/home/ale/test_jetauto_vc/src/extra_pkgs/compare_pose_est/bags/W_ConPID_Anti.bag"
       
        # Start playing the rosbag file
        play_process = subprocess.Popen(['rosbag', 'play', bag_path])
        
        # Keep the node running
        while not rospy.is_shutdown():
            if play_process.poll() is not None:
                self.plot()
                break
            self.run()
            rospy.sleep(0.1)      
           
        # Wait for the rosbag play process to finish
        # play_process.wait()

        
    def odom_gazebo(self, msg):
        self.posx_gazebo = msg.pose.pose.position.x
        self.posy_gazebo = msg.pose.pose.position.y
        x_gazebo.append(self.posx_gazebo)
        y_gazebo.append(self.posy_gazebo)
        
    def odom_sim(self, msg):
        self.posx_sim = msg.pose.pose.position.x
        self.posy_sim = msg.pose.pose.position.y
        x_sim.append(self.posx_sim)
        y_sim.append(self.posy_sim)

    def odom_real(self, msg):
        self.posx_real = msg.pose.pose.position.x
        self.posy_real = msg.pose.pose.position.y
        x_real.append(self.posx_real)
        y_real.append(self.posy_real)
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
            
    def plot(self):
        #Comparación trayectorias
        plt.figure(figsize=(15, 15))
        #plt.plot(x_gazebo, y_gazebo,label='Gazebo')
        plt.plot(x_sim, y_sim,label='Sim')
        plt.plot(x_real, y_real,label='Real')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Trayectoria')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Ensure aspect ratio is equal 
        plt.show() 

    def run(self):
        #Append pose sim para graficar en RVIZ
        self.pose = PoseStamped()
        self.pose.header = Header()
        self.pose.header.stamp = self.sim_time
        self.pose.header.frame_id = 'odom'
        self.pose.pose.position.x = self.posx_sim
        self.pose.pose.position.y = self.posy_sim
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.w = 1.0  # No rotation, quaternion format
        self.path_sim.poses.append(self.pose)
        
        self.path_sim_pub.publish(self.path_sim)
        
        #Append pose sim para graficar en RVIZ
        self.pose = PoseStamped()
        self.pose.header = Header()
        self.pose.header.stamp = self.sim_time
        self.pose.header.frame_id = 'odom'
        self.pose.pose.position.x = self.posx_real
        self.pose.pose.position.y = self.posy_real
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.w = 1.0  # No rotation, quaternion format
        self.path.poses.append(self.pose)
        
        self.path_pub.publish(self.path)
                     
if __name__ == "__main__":
    try:
        node = Compare_Pose_Est()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.play_process.terminate()
        sys.exit()
