#!/usr/bin/env python3

import os
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from jetauto_interfaces.msg import imu_encoder
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import tf.transformations
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

class Driver_Sensors:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('driver_sensors', anonymous=True)
        
        self.unfiltered_angular_speeds = [0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_speeds = [0.0, 0.0, 0.0, 0.0]
        self.filtered_angular_speeds_prev = [0.0, 0.0, 0.0, 0.0]
        self.n = 0  # Number of pulses to wait before computing speed
        self.max_delta_threshold = 600  # Threshold for sudden changes
        self.alpha = 0.2
        self.sim_time = rospy.Time.now()
        self.previous_time = rospy.Time.now()
        self.current_values = []
        self.previous_values = []


        # Publisher for the control output
        self.imu_encoder_pub = rospy.Publisher("/imu_encoder_sim", imu_encoder, queue_size=10)
        self.imu_encoder_msg = imu_encoder()
        
        rospy.Subscriber("/imu_data", Imu, self.imu_callback)
        rospy.Subscriber("/jetauto_wheel_speed_sim", Float32MultiArray, self.wheel_speed_callback)
        
        self.use_filter = rospy.get_param('use_filter', False)
        
        rospy.Subscriber('/clock', Clock, self.clock_callback)             
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
        

    def clock_callback(self, msg):
        self.sim_time = msg.clock
    
    def imu_callback(self, msg):
        self.imu_encoder_msg.imu = msg
        (r_imu,p_imu,y_imu) = tf.transformations.euler_from_quaternion((msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w))
        self.imu_encoder_msg.angle = y_imu
        
    def wheel_speed_callback(self, msg):
        if self.use_filter:
            self.unfiltered_angular_speeds = msg.data
            self.compute_speed_filtered()
            self.imu_encoder_msg.w1 = self.filtered_angular_speeds[0]
            self.imu_encoder_msg.w2 = self.filtered_angular_speeds[1]
            self.imu_encoder_msg.w3 = self.filtered_angular_speeds[2]
            self.imu_encoder_msg.w4 = self.filtered_angular_speeds[3] 
        else:
            self.imu_encoder_msg.w1 = msg.data[0]
            self.imu_encoder_msg.w2 = msg.data[1]
            self.imu_encoder_msg.w3 = msg.data[2]
            self.imu_encoder_msg.w4 = msg.data[3]
        self.imu_encoder_pub.publish(self.imu_encoder_msg)
        
        
    def compute_speed_filtered(self):
        current_time = self.sim_time				# Cambiar por el rospy.time.now()
        current_values = self.unfiltered_angular_speeds
        delta_time = (current_time - self.previous_time).to_sec()
        if delta_time > 0.005:
            for i in range(4):
                self.filtered_angular_speeds[i] = self.alpha * current_values[i] + (1 - self.alpha) * self.filtered_angular_speeds[i]           
            self.previous_time = current_time
        return self.filtered_angular_speeds

if __name__ == "__main__":
    try:
        sensor = Driver_Sensors()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

