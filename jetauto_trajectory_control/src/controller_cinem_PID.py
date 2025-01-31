#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import matplotlib.pyplot as plt
import Paths as paths
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Twist
from math import sqrt
import time
from std_msgs.msg import Float32MultiArray, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from jetauto_interfaces.msg import imu_encoder
from rosgraph_msgs.msg import Clock
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import numpy as np
from math import atan2

class PoseControl:
    def __init__(self):
        rospy.init_node("trajectory_controller", anonymous=False)
        
        # Load the parameters
        self.x_kp = rospy.get_param('/pid/x_kp', 1.95)
        self.x_ki = rospy.get_param('/pid/x_ki', 0.15)
        self.x_kd = rospy.get_param('/pid/x_kd', 0.0001)
        self.y_kp = rospy.get_param('/pid/y_kp', 1.75)
        self.y_ki = rospy.get_param('/pid/y_ki', 0.15)
        self.y_kd = rospy.get_param('/pid/y_kd', 0.0001)
        self.theta_kp = rospy.get_param('/pid/theta_kp', 2.2)
        self.theta_ki = rospy.get_param('/pid/theta_ki', 0.05)
        self.theta_kd = rospy.get_param('/pid/theta_kd', 0.0001)
        path_type = rospy.get_param('path_type', 'rectangle')
        self.r = rospy.get_param('r', '0.0485')
        self.lx = rospy.get_param('lx', '0.0975')
        self.ly = rospy.get_param('ly', '0.103')
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.guardar_datos = rospy.get_param('guardar_datos', False)
        
        if self.published_gazebo_odom:
            rospy.Subscriber('/odom', Odometry, self.odom_callback)
        else:
            rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_callback)
        
        rospy.Subscriber('/imu_encoder_sim', imu_encoder, self.imu_callback)
        rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)
        self.path_pub = rospy.Publisher('/path_ref', Path, queue_size=10)
        self.path_sim_pub = rospy.Publisher('/path_sim', Path, queue_size=10)
              
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
            
            
        #Para guardar datos en txt
        if self.guardar_datos:
            # Use rospkg to find the path to the package
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('jetauto_trajectory_control')
            # Construct the directory path
            directory = os.path.join(package_path,'datos','PID')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesnt exist
            self.file_name = os.path.join(directory, "PID_{}.txt".format(path_type))
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
      
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0
        self.w4 = 0.0
        
        self.t = []
        self.x_sim = []
        self.y_sim = []
        self.theta_sim = []
        self.w1_sim = []
        self.w2_sim = []
        self.w3_sim = []
        self.w4_sim = []
        self.w1_ref = []
        self.w2_ref = []
        self.w3_ref = []
        self.w4_ref = []
        
        self.acx = 0
        self.acy = 0
        self.acw = 0
                     
        self.e_x = [0.0, 0.0, 0.0]
        self.e_y = [0.0, 0.0, 0.0]
        self.e_theta = [0.0, 0.0, 0.0]
                     
        ##Definir trayectoria
        pth = paths.Paths()
        self.goalx    = pth.x
        self.goalx_d = pth.vx
        self.goaly    = pth.y
        self.goaly_d = pth.vy
        self.time     = pth.t
        self.tm       = pth.Ts
        self.goaltheta = pth.theta
        self.goaltheta_d = pth.w
        
        #Definir Path referencia RVIZ
        self.path = Path()
        self.path.header = Header()
        self.path.header.stamp = self.sim_time
        self.path.header.frame_id = 'odom'  # Frame of reference
        
        #Definir Path simulacion RVIZ
        self.path_sim = Path()
        self.path_sim.header = Header()
        self.path_sim.header.stamp = self.sim_time
        self.path_sim.header.frame_id = 'odom'  # Frame of reference
        
        for x, y in zip(self.goalx, self.goaly):
            self.pose = PoseStamped()
            self.pose.header = Header()
            self.pose.header.stamp = self.sim_time
            self.pose.header.frame_id = 'odom'
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = 0
            self.pose.pose.orientation.w = 1.0  # No rotation, quaternion format
            self.path.poses.append(self.pose)
            
        self.path.header.stamp = self.sim_time
        self.path_pub.publish(self.path)
        
        #Graficar trayectoria a seguir
        plt.figure(figsize=(10, 10))
        plt.plot(self.goalx, self.goaly)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trayectoria a seguir')
        plt.grid(True)
        circle = plt.Circle((self.goalx[0], self.goaly[0]), radius=0.05, color='b', alpha=1)
        plt.gca().add_patch(circle)
        plt.axis('equal')  # Ensure aspect ratio is equal 
        plt.show()
        
        #Calculo constantes para el controlador
        self.Ts = self.tm
        
        self.A0x = (2.0*self.Ts*self.x_kp + self.x_ki*pow(self.Ts,2) + 2.0*self.x_kd)/(2.0*self.Ts)
        self.A1x = (self.x_ki*pow(self.Ts,2) - 2.0*self.Ts*self.x_kp - 4.0*self.x_kd)/(2.0*self.Ts)
        self.A2x = self.x_kd/self.Ts
        
        self.A0y = (2.0*self.Ts*self.y_kp + self.y_ki*pow(self.Ts,2) + 2.0*self.y_kd)/(2.0*self.Ts)
        self.A1y = (self.y_ki*pow(self.Ts,2) - 2.0*self.Ts*self.y_kp - 4.0*self.y_kd)/(2.0*self.Ts)
        self.A2y = self.y_kd/self.Ts
        
        self.A0theta = (2.0*self.Ts*self.theta_kp + self.theta_ki*pow(self.Ts,2) + 2.0*self.theta_kd)/(2.0*self.Ts)
        self.A1theta = (self.theta_ki*pow(self.Ts,2) - 2.0*self.Ts*self.theta_kp - 4.0*self.theta_kd)/(2.0*self.Ts)
        self.A2theta = self.theta_kd/self.Ts

    def imu_callback(self, msg):
        self.theta = msg.angle
        self.w1 = msg.w1
        self.w2 = msg.w2
        self.w3 = msg.w3
        self.w4 = msg.w4

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
    def append_data(self, t,x,y,theta,w1_sim,w2_sim,w3_sim,w4_sim,w1_ref,w2_ref,w3_ref,w4_ref):
        self.t.append(t)
        self.x_sim.append(x)
        self.y_sim.append(y)
        self.theta_sim.append(theta)
        self.w1_sim.append(w1_sim)
        self.w2_sim.append(w2_sim)
        self.w3_sim.append(w3_sim)
        self.w4_sim.append(w4_sim)
        self.w1_ref.append(w1_ref)
        self.w2_ref.append(w2_ref)
        self.w3_ref.append(w3_ref)
        self.w4_ref.append(w4_ref)

    def write_file(self, t, x, y, theta, x_sim, y_sim, theta_sim, w1_sim, w2_sim, w3_sim, w4_sim, w1_ref, w2_ref, w3_ref, w4_ref):
        with open(self.file_name, "a") as file:
            for i in range(0,len(self.goalx)): 
                file.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(t[i], x[i], y[i], theta[i], x_sim[i], y_sim[i], theta_sim[i], w1_sim[i], w2_sim[i], w3_sim[i], w4_sim[i], w1_ref[i], w2_ref[i], w3_ref[i], w4_ref[i]))
        
    def plot(self):
        win_size_x = 15
        win_size_y = 10   
        #Error x
        #plt.plot(self.t,self.x_error)
        #plt.xlabel('time')
        #plt.ylabel('x error')
        #plt.title('X Error')
        #plt.grid(True)
        #plt.show()
        
        #Error y
        #plt.plot(self.t,self.y_error)
        #plt.xlabel('time')
        #plt.ylabel('y error')
        #plt.title('Y Error')
        #plt.grid(True)
        #plt.show()
        
        #Comparacion trayectorias
        plt.figure(figsize=(win_size_y, win_size_y))
        plt.plot(self.goalx, self.goaly,label='Referencia')
        plt.plot(self.x_sim,self.y_sim,label='Simulacion')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Trayectoria')
        plt.grid(True)
        plt.legend()
        circle = plt.Circle((self.x_sim[-1], self.y_sim[-1]), radius=0.05, color='tab:orange', alpha=1)
        plt.gca().add_patch(circle)
        plt.axis('equal')  # Ensure aspect ratio is equal 
        plt.show() 
        
        #Senales de Contol
        #plt.plot(self.t, self.w1,label='w1')
        #plt.plot(self.t,self.w2,label='w2')
        #plt.plot(self.t, self.w3,label='w3')
        #plt.plot(self.t,self.w4,label='w4')
        #plt.xlabel('x')
        #plt.ylabel('y')
        #plt.title('Velocidad ruedas')
        #plt.grid(True)
        #plt.legend() 
        #plt.show()
    
    def get_inv_Jacobian(self,th):
        th1 = th + np.pi/4
        r2 = np.sqrt(2)
        J_inv = np.array([[r2 * np.cos(th1) , r2 * np.sin(th1), -(self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1), -(self.lx + self.ly)],
                          [r2 * np.cos(th1) , r2 * np.sin(th1),  (self.lx + self.ly)],
                          [r2 * np.sin(th1) ,-r2 * np.cos(th1),  (self.lx + self.ly)]])
        return J_inv
    
    def run(self):
        init_time = self.sim_time
        last_time = init_time
        
        for i in range(0,len(self.goalx)):
            
            while not rospy.is_shutdown() and (self.sim_time-init_time).to_sec() < self.time[i]:
                pass 
                
            last_time = self.sim_time
            
            self.e_x[0] = self.goalx[i] - self.x
            self.e_y[0] = self.goaly[i] - self.y
            self.e_theta[0] = self.goaltheta[i] - self.theta
            self.e_theta[0] = (self.e_theta[0] + np.pi) % (2*np.pi) - np.pi
                       
            self.acx = self.acx + self.A0x * self.e_x[0] + self.A1x * self.e_x[1] + self.A2x * self.e_x[2]
            self.acy = self.acy + self.A0y * self.e_y[0] + self.A1y * self.e_y[1] + self.A2y * self.e_y[2]
            self.acw = self.acw + self.A0theta * self.e_theta[0] + self.A1theta * self.e_theta[1] + self.A2theta * self.e_theta[2]
            
            self.e_x[2] = self.e_x[1]
            self.e_x[1] = self.e_x[0]
            
            self.e_y[2] = self.e_y[1]
            self.e_y[1] = self.e_y[0]
            
            self.e_theta[2] = self.e_theta[1]
            self.e_theta[1] = self.e_theta[0]
            
            u = np.array([[self.acx],[self.acy],[self.acw]])
            J_inv = self.get_inv_Jacobian(self.theta)
            w = np.dot(J_inv,u)/self.r
            w1_aux = w[0,0]
            w2_aux = w[1,0]
            w3_aux = w[2,0]
            w4_aux = w[3,0]
            a = 9.00
            w1 = max(min(w1_aux, a), -a)
            w2 = max(min(w2_aux, a), -a)
            w3 = max(min(w3_aux, a), -a)
            w4 = max(min(w4_aux, a), -a)
            
            self.append_data((last_time-init_time).to_sec(),self.x,self.y,self.theta,self.w1,self.w2,self.w3,self.w4,w1,w2,w3,w4)
            
            # Publish the wheels message
            self.control_publisher.publish(Float32MultiArray(data=[w1, w2, w3, w4]))
            
            #Append pose sim para graficar en RVIZ
            self.pose = PoseStamped()
            self.pose.header = Header()
            self.pose.header.stamp = self.sim_time
            self.pose.header.frame_id = 'odom'
            self.pose.pose.position.x = self.x
            self.pose.pose.position.y = self.y
            self.pose.pose.position.z = 0
            self.pose.pose.orientation.w = 1.0  # No rotation, quaternion format
            self.path_sim.poses.append(self.pose)
            
            self.path_sim_pub.publish(self.path_sim)
            
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        if self.guardar_datos:
                self.write_file(self.t,self.goalx, self.goaly, self.goaltheta, self.x_sim, self.y_sim, self.theta_sim, self.w1_sim, self.w2_sim, self.w3_sim, self.w4_sim, self.w1_ref, self.w2_ref, self.w3_ref, self.w4_ref)
            
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
