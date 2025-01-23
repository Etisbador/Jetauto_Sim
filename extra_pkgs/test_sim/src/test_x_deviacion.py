#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import matplotlib.pyplot as plt
import Paths as paths
import numpy as np
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
from jetauto_interfaces.msg import imu_encoder


class PoseControl:
    def __init__(self):
        rospy.init_node("trajectory_controller", anonymous=False)
        
        self.d = 96.5/(1000.0)
        self.lx = 9.75/100.0
        self.ly = 10.3/100.0
        
        # Load the parameters
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.guardar_datos = rospy.get_param('guardar_datos', False)
        self.usar_PID = rospy.get_param('usar PID', False)
        
        if self.published_gazebo_odom:
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        else:
            rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_callback)
            
        if self.usar_PID:
            self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)
        else:
            self.control_publisher = rospy.Publisher("jetauto_wheel_cmd_sim", Float32MultiArray, queue_size=10)
        
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        rospy.Subscriber('/imu_encoder_sim', imu_encoder, self.imu_encoder_callback)
        
              
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
            
            
        #Para guardar datos en txt
        if self.guardar_datos:
            # Use rospkg to find the path to the package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('test_sim')
            # Construct the directory path
            directory = os.path.join(package_path,'datos','x_desv')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesn't exist
            self.file_name = os.path.join(directory, f"X_Desv.txt")
            with open(self.file_name, "w") as file:
                pass
            
        #Iniciar posición del robot
        
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        
        rospy.sleep(1.5)
            
        #Iniciar posición del robot
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
      
        self.x = 0.0
        self.y = 0.0
        self.x_vect = []
        self.y_vect = []
        self.t = []
        
        self.tm = 0.05
        self.tf = 10
                                         
        pth = paths.Paths(self.tm, self.tf)
        self.time = pth.time()
        
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
    def write_file(self, t, theta_odom, theta_sim):
        with open(self.file_name, "a") as file:
            file.write(f"{t}\t{theta_odom}\t{theta_sim}")
            file.write("\n")
            
    def imu_encoder_callback(self, msg):
        self.theta_imu = msg.angle       # angulo en rads  
        
    def plot(self):
        win_size_x = 15
        win_size_y = 10      
               
        #Comparación trayectorias
        plt.figure(figsize=(win_size_y, win_size_y))
        plt.plot(self.t, self.theta_odom_vect,label='Odom')
        plt.plot(self.t,self.theta_imu_vect,label='Imu')
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Angulo [rad]')
        plt.title('Comparacion angulo')
        plt.grid(True)
        plt.legend()
        plt.show() 

    
    def run(self):
    
        ang = 0.89*2.0*np.pi/self.tf
        w = (ang * (self.lx + self.ly)) / (self.d/2.0);
        
        init_time = self.sim_time
        last_time = init_time
        
        
        for i in range(0,len(self.time)):
            
            while not rospy.is_shutdown() and (self.sim_time-init_time).to_sec() < self.time[i]:
                pass 
                
            last_time = self.sim_time

            self.control_publisher.publish(Float32MultiArray(data=[-w, -w, w, w]))
                          
            self.t.append((last_time-init_time).to_sec())
            self.theta_odom_vect.append(self.theta_odom)
            self.theta_imu_vect.append(self.theta_imu)

            
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.theta_odom, self.theta_imu)

        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        self.plot()

if __name__ == "__main__":
    try:
        node = PoseControl()
        node.run()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        sys.exit()
