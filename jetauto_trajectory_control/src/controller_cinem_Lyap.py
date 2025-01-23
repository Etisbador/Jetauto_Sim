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
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Wrench

class PoseControl:
    def __init__(self):
        rospy.init_node("trajectory_controller", anonymous=False)

        #Load parameters
        self.kp = rospy.get_param('cinem_Lyap/kp', 1.9)
        path_type = rospy.get_param('path_type', 'rectangle')
        self.tm = rospy.get_param('tiempo_muestreo', '0.02')
        self.tf = rospy.get_param('tiempo_total', '60')
        self.r = rospy.get_param('r', '0.0485')
        self.lx = rospy.get_param('lx', '0.0975')
        self.ly = rospy.get_param('ly', '0.103')
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.guardar_datos = rospy.get_param('guardar_datos', False)
                              
        #if self.published_gazebo_odom:
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_sim_callback)
        #else:
            #rospy.Subscriber('/jetauto_odom_sim', Odometry, self.odom_callback)
        
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
            directory = os.path.join(package_path,'datos','Pert')
            if not os.path.exists(directory):
                os.makedirs(directory)  # Create the directory if it doesn't exist
            self.file_name = os.path.join(directory, f"Pert_Est.txt")
            with open(self.file_name, "w") as file:
                pass
        
        
        #Iniciar posición del robot
        
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
            
        self.x = 0
        self.y = 0
        self.theta = 0.0
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0.0
        self.e_x_ant = 0
        self.e_y_ant = 0
        self.e_theta_ant = 0.0
        self.x_error = []
        self.y_error = []
        self.theta_error = []
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
        
        if path_type == 'ellipse':
            #Circulo/Elipse
            a = 1   #eje principal (x)
            b = 1   #ejesecundario (y)
            center = [0, -b]   # centro
            self.goalx, self.goaly = pth.ellipse(a, b, center)
            self.goalx_d, self.goaly_d = pth.ellipse_d(a, b, center)
        elif path_type == 'lemniscate':
            #Lemniscate/Forma 8
            a = 2   #factor de escalamiento
            center = [0, 0]
            self.goalx, self.goaly = pth.lemniscate(a, center)
            self.goalx_d, self.goaly_d = pth.lemniscate_d(a, center)
        elif path_type == 'spiral':
            #Espiral
            a = 0.4   #radio inicial
            b = 0.04   #razon de crecimiento
            center = [0, -a]
            self.goalx, self.goaly = pth.spiral(a, b, center)
            self.goalx_d, self.goaly_d = pth.spiral_d(a, b, center)
        elif path_type == 'line':
            #Linea
            start = [0, 0]   #coordenada inicial
            end = [3, 5]     #coordenada final
            self.goalx, self.goaly = pth.line(start, end)
            self.goalx_d, self.goaly_d = pth.line_d(start, end)
        elif path_type == 'sine':
            #Sine
            A = 1     #amplitud
            f = 1/self.tf   #frecuencia
            n = 5     # atenuación de crecimiento en x (center_x + t/n)
            center = [0, 0]    #centro
            self.goalx, self.goaly = pth.sine(A, f*self.tf, n ,center)
            self.goalx_d, self.goaly_d = pth.sine_d(A, f*self.tf, n ,center)
        elif path_type == 'rectangle':
            #Rectangulo
            length = 3   #base (x)
            width = 2    #altura (y)
            center = [0, 0]   # centro
            self.goalx, self.goaly = pth.rectangle(length,width,center)
            self.goalx_d, self.goaly_d= pth.rectangle_d(length,width,center)
        else:
            rospy.logwarn("Tipo de trayectoria desconocida: %s" % path_type)
            self.goalx, self.goaly = [], []
        
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
        
        self.run()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])                     
        
    def odom_sim_callback(self, msg):
        self.x_est = msg.pose.pose.position.x
        self.y_est = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta_est) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])      
        
    def clock_callback(self, msg):
        self.sim_time = msg.clock
        
    def calculate_ISE(self, errors):
        # Square each error value
        squared_errors = np.square(errors)
        # Integrate the squared errors (numerical integration using the trapezoidal rule)
        ise = np.trapz(squared_errors)
        return ise
        
    def write_file(self, t, x, y, theta, x_sim, y_sim, theta_sim, e_x, e_y, e_theta, w1, w2, w3, w4):
        with open(self.file_name, "a") as file:
            file.write(f"{t}\t{x}\t{y}\t{theta}\t{x_sim}\t{y_sim}\t{theta_sim}\t{x_sim}\t{y_sim}\t{theta_sim}\t{w1}\t{w2}\t{w3}\t{w4}")
            file.write("\n")
        
    def plot(self):
        win_size_x = 15
        win_size_y = 10             
               
        #Comparación trayectorias
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
             
        for i in range(0,len(self.goalx)):
            
            if i > 0:
                while not rospy.is_shutdown() and (self.sim_time-init_time).to_sec() < self.time[i]:
                    pass 

            last_time = self.sim_time
            
            dt = (self.sim_time-last_time).to_sec()
            
            #if self.published_gazebo_odom:
            #    e_x = self.goalx[i] - self.x
            #    e_y = self.goaly[i] - self.y
            #    e_theta = ref_theta - self.theta
            #else:
            e_x = self.goalx[i] - self.x_est
            e_y = self.goaly[i] - self.y_est
            e_theta = ref_theta - self.theta_est   
            
            ei_x = self.e_x_ant + e_x * dt
            ei_y = self.e_y_ant + e_y * dt
            ei_theta = self.e_theta_ant + e_theta * dt
            
            acx = (self.kp * e_x + self.goalx_d[i])
            acy = (self.kp * e_y + self.goaly_d[i])
            acw = (self.kp * e_theta)
            
            self.e_x_ant = e_x
            self.e_y_ant = e_y
            self.e_theta_ant = e_theta
            
            self.x_sim.append(self.x)
            self.y_sim.append(self.y)          
            
            theta_1 = self.theta + np.pi / 4
            J1 = [np.sqrt(2)*np.cos(theta_1),np.sqrt(2)*np.sin(theta_1),np.sqrt(2)*np.cos(theta_1),np.sqrt(2)*np.sin(theta_1)]
            J2 = [np.sqrt(2)*np.sin(theta_1),-np.sqrt(2)*np.cos(theta_1),np.sqrt(2)*np.sin(theta_1),-np.sqrt(2)*np.cos(theta_1)]
            J3 = [-1/(self.lx + self.ly), -1/(self.lx + self.ly), 1/(self.lx + self.ly), 1/(self.lx + self.ly)]
            J_array = np.array([J1,J2,J3])
            J = (self.r/4) * J_array
            J_inv = np.linalg.pinv(J)
            
            ac_vector = np.array([[acx],[acy],[acw]])
            
            W = J_inv @ ac_vector        
            
            w1_aux,w2_aux,w3_aux,w4_aux = W.flatten()

            w1 = max(min(w1_aux, 9), -9)
            w2 = max(min(w2_aux, 9), -9)
            w3 = max(min(w3_aux, 9), -9)
            w4 = max(min(w4_aux, 9), -9)
            
            self.w1.append(w1)
            self.w2.append(w2)
            self.w3.append(w3)
            self.w4.append(w4)
                            
            # Publish the stop message
            self.control_publisher.publish(Float32MultiArray(data=[w1, w2, w3, w4]))
                          
            #print([e_x,e_y,(last_time-init_time).to_sec()])
            self.x_error.append(e_x)
            self.y_error.append(e_y)
            self.theta_error.append(e_theta)
            self.t.append((last_time-init_time).to_sec())
            
            if i == int(len(self.goalx)*1.0/8.0):        
            
                rospy.wait_for_service('/gazebo/apply_body_wrench')
                try:
                    apply_wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
                    
                    # Create a request object
                    request = ApplyBodyWrenchRequest()
                    request.body_name = "jetauto::base_footprint"  # Specify the link name
                    request.reference_frame = "world"  # Apply force in the world frame
                    request.wrench.force.x = 32.5
                    request.wrench.force.y = 0  # Force in the Y-axis
                    request.wrench.force.z = 0
                    request.start_time = rospy.Time.now()  # Start applying the force immediately
                    request.duration = rospy.Duration(1)  # Apply the force for 1 second

                    # Call the service
                    response = apply_wrench_service(request)
                    if response.success:
                        rospy.loginfo("Force applied successfully!")
                    else:
                        rospy.logwarn("Failed to apply force.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")   
                    
            if i == int(len(self.goalx)*5.0/8.0):        
            
                rospy.wait_for_service('/gazebo/apply_body_wrench')
                try:
                    apply_wrench_service = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
                    
                    # Create a request object
                    request = ApplyBodyWrenchRequest()
                    request.body_name = "jetauto::base_footprint"  # Specify the link name
                    request.reference_frame = "world"  # Apply force in the world frame
                    request.wrench.force.x = 0
                    request.wrench.force.y = 32.5  # Force in the Y-axis
                    request.wrench.force.z = 0
                    request.start_time = rospy.Time.now()  # Start applying the force immediately
                    request.duration = rospy.Duration(1)  # Apply the force for 1 second

                    # Call the service
                    response = apply_wrench_service(request)
                    if response.success:
                        rospy.loginfo("Force applied successfully!")
                    else:
                        rospy.logwarn("Failed to apply force.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")  
            
            
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
            
            if self.guardar_datos:
                self.write_file((last_time-init_time).to_sec(),self.goalx[i], self.goaly[i], ref_theta, self.x, self.y, self.theta, self.x_est, self.y_est, self.theta_est, w1, w2, w3, w4)
            
            
        self.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        self.plot()

if __name__ == "__main__":
    try:
        node = PoseControl()
        #node.run()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.control_publisher.publish(Float32MultiArray(data=[0, 0, 0, 0]))
        sys.exit()
