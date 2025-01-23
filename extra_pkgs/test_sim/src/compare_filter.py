#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import matplotlib.pyplot as plt
import Paths as paths
import numpy as np
from std_msgs.msg import Float32MultiArray, Header
from jetauto_interfaces.msg import imu_encoder
from rosgraph_msgs.msg import Clock
import Paths as paths
import matplotlib.pyplot as plt
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist

class WheelSpeedController:
    def __init__(self):
        rospy.init_node("wheel_speed_controller", anonymous=False)
        
        self.tm = 0.005
        self.tf = 2.0
        
        # Read current wheel speeds
        self.current_speeds = [0,0,0,0]
        self.w1 = []
        self.w2 = []
        self.w3 = []
        self.w4 = []
        self.t = []
                                         
        pth = paths.Paths(self.tm, self.tf)
        self.time = pth.time()
        
        # Load the parameters
        self.guardar_datos = rospy.get_param('guardar_datos', False)
        
        # Publisher for the control output
        self.control_publisher = rospy.Publisher("jetauto_wheel_cmd_sim", Float32MultiArray, queue_size=10)
        
        # Subscriber for the wheel speed
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
            directory = os.path.join(package_path,'datos','filter')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesn't exist
            self.file_name = os.path.join(directory, f"filter.txt")
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
                
            
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
    def imu_encoder_callback(self, msg):
        # Read current wheel speeds
        self.current_speeds = [msg.w1, msg.w2, msg.w3, msg.w4]
    
    def plot(self):
        win_size_x = 15
        win_size_y = 10             
        
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
    
    def write_file(self, t, w1, w2, w3 ,w4):
        with open(self.file_name, "a") as file:
            file.write(f"{t}\t{w1}\t{w2}\t{w3}\t{w4}")
            file.write("\n")
    
    def run(self):
        init_time = self.sim_time
        last_time = init_time
        a = 2.0
        
        for i in range(0,len(self.time)):
            
            while not rospy.is_shutdown() and (self.sim_time-init_time).to_sec() < self.time[i]:
                pass
            
            
            if i+1 >= (len(self.time)*1.0/8.0):
                control_msg = Float32MultiArray(data=[a, a, a, a])
                self.control_publisher.publish(control_msg)
                print(control_msg)
            else:
                control_msg = Float32MultiArray(data=[0, 0, 0, 0])
                self.control_publisher.publish(control_msg)
                print(control_msg)
            
            last_time = self.sim_time
            
            self.w1.append(self.current_speeds[0])
            self.w2.append(self.current_speeds[1])
            self.w3.append(self.current_speeds[2])
            self.w4.append(self.current_speeds[3])
            self.t.append((last_time-init_time).to_sec())
            
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.current_speeds[0],self.current_speeds[1],self.current_speeds[2],self.current_speeds[3])

        control_msg = Float32MultiArray(data=[0.0,0.0,0.0,0.0])
        #Publish the control outputs
        self.control_publisher.publish(control_msg)
        print(control_msg)
        self.plot()
        
if __name__ == "__main__":
    try:
        node = WheelSpeedController()
        node.run()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        sys.exit()
