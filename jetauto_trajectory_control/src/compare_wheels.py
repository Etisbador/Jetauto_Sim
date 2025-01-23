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
        
        # Load the parameters
        self.tm = rospy.get_param('tiempo_muestreo', '0.1')
        self.tf = rospy.get_param('tiempo_total', '15')
        self.r = rospy.get_param('r', '0.0485')
        self.lx = rospy.get_param('lx', '0.0975')
        self.ly = rospy.get_param('ly', '0.103')
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.guardar_datos = rospy.get_param('guardar_datos', False)
        
        self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)
        
        rospy.Subscriber("/imu_encoder_sim", imu_encoder, self.speed_callback)
        
        if self.published_gazebo_odom:
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        else:
            rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_callback)
            
        rospy.Subscriber('/clock', Clock, self.clock_callback)
              
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
            
            
        #Para guardar datos en txt
        if self.guardar_datos:
            # Use rospkg to find the path to the package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('jetauto_trajectory_control')
            # Construct the directory path
            directory = os.path.join(package_path,'datos','comp_wheels')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesn't exist
            self.file_name = os.path.join(directory, f"PID_{path_type}.txt")
            with open(self.file_name, "w") as file:
                pass
            
        #Iniciar posición del robot
        
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        
        rospy.sleep(2.5)
            
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
      
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.x_sim = []
        self.y_sim = []
        self.w1 = []
        self.w2 = []
        self.w3 = []
        self.w4 = []
        self.t = []
                     
        ##Definir trayectoria
        pth = paths.Paths(self.tm, self.tf)
        self.time = pth.time()
        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
    def speed_callback(self, msg):
        # Read current wheel speeds
        self.current_speeds = [msg.w1, msg.w2, msg.w3, msg.w4]
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
        
    def write_file(self, t, x_sim, y_sim, theta_sim, w1, w2, w3, w4):
        with open(self.file_name, "a") as file:
            file.write(f"{t}\t{x_sim}\t{y_sim}\t{theta_sim}\t{w1}\t{w2}\t{w3}\t{w4}")
            file.write("\n")
        
    def plot(self):
        win_size_x = 15
        win_size_y = 10             
               
        #Comparación trayectorias
        plt.figure(figsize=(win_size_y, win_size_y))
        plt.plot(self.x_sim,self.y_sim)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Trayectoria')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Ensure aspect ratio is equal 
        plt.show() 
        
        #Señales de control
        plt.figure(figsize=(win_size_x, win_size_y))
        plt.plot(self.t, self.w1,label='w1',linewidth=0.5)
        plt.plot(self.t, self.w2,label='w2',linewidth=0.5)
        plt.plot(self.t, self.w3,label='w3',linewidth=0.5)
        plt.plot(self.t, self.w4,label='w4',linewidth=0.5)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Velocidad [rad/s]')
        plt.title('Señal de control ruedas')
        plt.grid(True)
        plt.legend()
        plt.show() 
    
    def run(self):
        ref_theta = 0
        init_time = self.sim_time
        last_time = init_time
        a = 2.5
        stop_length = 0.08  #porcentaje
        for i in range(0,len(self.time)):
            
            while not rospy.is_shutdown() and (self.sim_time-init_time).to_sec() < self.time[i]:
                pass

            last_time = self.sim_time
            if i > (len(self.time)*3/4):
                if i > ((1-stop_length)*len(self.time)):
                    setpoint = Float32MultiArray(data=[0, 0, 0, 0])
                else:
                    setpoint = Float32MultiArray(data=[a, -a, a, -a])
            elif i > (len(self.time)*2/4):
                if i > ((3/4-stop_length)*len(self.time)):
                    setpoint = Float32MultiArray(data=[0, 0, 0, 0])
                else:
                    setpoint = Float32MultiArray(data=[-a, -a, -a, -a])
            elif i > (len(self.time)*1/4):
                if i > ((2/4-stop_length)*len(self.time)):
                    setpoint = Float32MultiArray(data=[0, 0, 0, 0])
                else:
                    setpoint = Float32MultiArray(data=[-a, a, -a, a])
            else:
                if i > ((1/4-stop_length)*len(self.time)):
                    setpoint = Float32MultiArray(data=[0, 0, 0, 0])
                else:
                    setpoint = Float32MultiArray(data=[a, a, a, a])
            
            # Publish the stop message
            self.control_publisher.publish(setpoint)
                          
            #print([setpoint,i])
            self.x_sim.append(self.x)
            self.y_sim.append(self.y)
            #self.theta_sim.append(self.theta)
            self.w1.append(self.current_speeds[0])
            self.w2.append(self.current_speeds[1])
            self.w3.append(self.current_speeds[2])
            self.w4.append(self.current_speeds[3])
            self.t.append((last_time-init_time).to_sec())
            
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.x, self.y, self.theta, w1, w2, w3, w4)

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
