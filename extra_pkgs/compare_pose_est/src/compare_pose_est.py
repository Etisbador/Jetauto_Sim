#!/usr/bin/env python3

import rospy
import time
import subprocess
import rosbag
import matplotlib.pyplot as plt
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


x_real=[]
y_real=[]
x_sim=[]
y_sim=[]
x_est=[]
y_est=[]


class Compare_Pose_Est():
    def __init__(self):
        rospy.init_node('compare_pose_est', anonymous=False)   
        
        rospy.Subscriber("/odom", Odometry, self.odom_sim)
        rospy.Subscriber("/odom_jetauto", Odometry, self.odom_est)
        rospy.Subscriber("/jetauto_odom", Odometry, self.odom_real)
        rospy.Subscriber("/filtered_angular_speed", Float32MultiArray, self.speed_real)
        
        self.control_publisher = rospy.Publisher("wheel_vel", Float32MultiArray, queue_size=10)  
        #self.control_publisher = rospy.Publisher("wheel_setpoint", Float32MultiArray, queue_size=10)  
        
        self.posx_sim = 0.0
        self.posx_real = 0.0
        self.posx_est = 0.0
        self.posy_sim = 0.0
        self.posy_real = 0.0
        self.posy_est = 0.0
        
        
        # Iniciar la posición del roboto en gazebo
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            model_state = ModelState()
            model_state.model_name = 'jetauto1'
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
        
        bag_path = "/home/ale/test_jetauto_vc/src/compare_pose_est/Prueba_XY.bag"
    
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
        
    def odom_sim(self, msg):
        self.posx_sim = msg.pose.pose.position.x
        self.posy_sim = msg.pose.pose.position.y
        x_sim.append(self.posx_sim)
        y_sim.append(self.posy_sim)
        
    def odom_est(self, msg):
        self.posx_est = msg.pose.pose.position.x
        self.posy_est = msg.pose.pose.position.y
        x_est.append(self.posx_est)
        y_est.append(self.posy_est)

    def odom_real(self, msg):
        self.posx_real = msg.pose.pose.position.x
        self.posy_real = msg.pose.pose.position.y
        x_real.append(self.posx_real)
        y_real.append(self.posy_real)
        
    def speed_real(self, msg):
        wheel_speed = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        wheel_speed.data[0] = -msg.data[0]
        wheel_speed.data[1] = msg.data[1]
        wheel_speed.data[2] = -msg.data[2]
        wheel_speed.data[3] = msg.data[3]
        
        self.control_publisher.publish(wheel_speed)
    
    def plot(self):
        mysignals = [{'name': 'Odom Sim', 'x': x_sim,
                     'y': y_sim, 'color':'r', 'linewidth':1},
                    {'name': 'Odom_Est_Sim', 'x': x_est,
                     'y': y_est, 'color':'b', 'linewidth':3},
                    {'name': 'Odom_Real', 'x': x_real,
                     'y': y_real, 'color':'g', 'linewidth':3}]
             
        fig, ax = plt.subplots()
        for signal in mysignals:
            ax.plot(signal['x'], signal['y'],
                    color=signal['color'], 
                    linewidth=signal['linewidth'],
                    label=signal['name'])

        # Enable legend
        ax.legend()
        ax.set_title("Comparación pose")
        plt.show()
                     
if __name__ == "__main__":
    try:
        node = Compare_Pose_Est()
        
    except Exception as e:
        rospy.logerr(str(e))
