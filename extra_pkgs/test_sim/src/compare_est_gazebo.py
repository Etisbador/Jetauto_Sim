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
        self.tm = rospy.get_param('tiempo_muestreo', 0.05)
        self.tf = rospy.get_param('tiempo_total', 60.0)
        self.r = rospy.get_param('r', '0.0485')
        self.lx = rospy.get_param('lx', '0.0975')
        self.ly = rospy.get_param('ly', '0.103')
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.guardar_datos = rospy.get_param('guardar_datos', False)
        
        
        self.x = 0
        self.y = 0
        self.x_odom = 0
        self.y_odom = 0
        self.theta = 0.0
        self.x_sim = []
        self.y_sim = []
        self.x_gaz = []
        self.y_gaz = []
        self.w1 = []
        self.w2 = []
        self.w3 = []
        self.w4 = []
        self.t = []
        self.raw_speeds = []
        
        self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)
        
        
        rospy.Subscriber("/imu_encoder_sim", imu_encoder, self.speed_callback)
        
        rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback_gazebo)
        rospy.Subscriber("/jetauto_wheel_speed_sim", Float32MultiArray, self.wheel_speed_callback)
            
        rospy.Subscriber('/clock', Clock, self.clock_callback)
              
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
            
            
        #Para guardar datos en txt
        if self.guardar_datos:
            # Use rospkg to find the path to the package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('test_sim')
            # Construct the directory path
            directory = os.path.join(package_path,'datos','PID_comp')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesn't exist
            self.file_name = os.path.join(directory, f"0.2_1.95_4.8_0.0.txt")
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
                     
        ##Definir trayectoria
        pth = paths.Paths(self.tm, self.tf)
        self.time = pth.time()
        
    def wheel_speed_callback(self, msg):
        self.raw_speeds = msg.data
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
    def odom_callback_gazebo(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
    
    def speed_callback(self, msg):
        # Read current wheel speeds
        self.current_speeds = [msg.w1, msg.w2, msg.w3, msg.w4]
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
        
    def write_file(self, t, x_sim, y_sim, x_gaz, y_gaz,w1,w2,w3,w4):
    #def write_file(self, t, w1,w2,w3,w4, w1_raw,w2_raw,w3_raw,w4_raw):
        with open(self.file_name, "a") as file:
            #file.write(f"{t}\t{w1}\t{w2}\t{w3}\t{w4}\t{w1_raw}\t{w2_raw}\t{w3_raw}\t{w4_raw}")
            file.write(f"{t}\t{x_sim}\t{y_sim}\t{x_gaz}\t{y_gaz}\t{w1}\t{w2}\t{w3}\t{w4}")
            file.write("\n")
        
    def plot(self):
        win_size_x = 15
        win_size_y = 10             
               
        #Comparación trayectorias
        plt.figure(figsize=(win_size_y, win_size_y))
        plt.plot(self.x_sim,self.y_sim,label='Sim')
        plt.plot(self.x_gaz,self.y_gaz,label='Gazebo')
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
        stop_length = 0.05  #porcentaje
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
                #break
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
                          
            self.x_sim.append(self.x)
            self.y_sim.append(self.y)
            self.x_gaz.append(self.x_odom)
            self.y_gaz.append(self.y_odom)
            self.w1.append(self.current_speeds[0])
            self.w2.append(self.current_speeds[1])
            self.w3.append(self.current_speeds[2])
            self.w4.append(self.current_speeds[3])
            self.t.append((last_time-init_time).to_sec())
            
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.x, self.y, self.x_odom, self.y_odom,self.current_speeds[0],self.current_speeds[1],self.current_speeds[2],self.current_speeds[3])
                #self.write_file((last_time-init_time).to_sec(),self.current_speeds[0],self.current_speeds[1],self.current_speeds[2],self.current_speeds[3],self.raw_speeds[0],self.raw_speeds[1],self.raw_speeds[2],self.raw_speeds[3])

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
