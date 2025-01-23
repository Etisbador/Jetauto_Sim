#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import rospy
import tf.transformations
import math
import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from jetauto_interfaces.msg import imu_encoder
from std_msgs.msg import Float32MultiArray
from rosgraph_msgs.msg import Clock

class Compute_Vel_Pose:
    def __init__(self):
        rospy.init_node('pose_estimation', anonymous=False)
        rospy.Subscriber('/imu_encoder_sim', imu_encoder, self.imu_encoder_callback)
        self.odom = Odometry() 
        self.imu_encoder_msg = imu_encoder() #este mensaje se llama imu_encoder
        self.published_gazebo_odom = rospy.get_param('publish_gazebo_odom', True)
        self.br = tf2_ros.TransformBroadcaster()
        
        # Subscribe to the /clock topic
        rospy.Subscriber('/clock', Clock, self.clock_callback)
     
        #constantes del robot
        self.r = rospy.get_param("r", 0.0485)
        self.lx = rospy.get_param("lx", 0.0975)
        self.ly = rospy.get_param("ly", 0.103)
        
        # para el calculo de la posicion
        self.sim_time = rospy.Time.now()
        self.time_actual = rospy.Time.now()
        self.time_ant = rospy.Time.now()
        self.vx_ant = 0
        self.vy_ant = 0
        self.wz_ant = 0
        self.px_ant = 0
        self.py_ant = 0
        self.thetam = 0
        self.theta_ant = 0
        
        if not rospy.get_param('/use_sim_time', False):
            rospy.set_param('/use_sim_time', True)
        
        self.odom_pub = rospy.Publisher("/jetauto_odom_sim", Odometry, queue_size=2)
                   
    def clock_callback(self, msg):
        self.sim_time = msg.clock
      
    def imu_encoder_callback(self, msg):
           
        self.time_actual = self.sim_time
        dt = (self.time_actual- self.time_ant).to_sec()
        self.time_ant = self.time_actual
        
        w1 = msg.w1
        w2 = msg.w2
        w3 = msg.w3
        w4 = msg.w4
        yaw = msg.angle       # angulo en rads       
                      
        a = 4*(self.lx+self.ly)
        vx = (w1+w2+w3+w4)*self.r/4
        vy = (w1-w2+w3-w4)*self.r/4
               
        vxw = vx*math.cos(yaw)-vy*math.sin(yaw)
        vyw = vx*math.sin(yaw)+vy*math.cos(yaw)
        
        px = self.px_ant+(vxw+self.vx_ant)*dt/2.0
        py = self.py_ant+(vyw+self.vy_ant)*dt/2.0
        theta = yaw
       
        self.odom.pose.pose.position.x = px
        self.odom.pose.pose.position.y = py
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.odom.pose.pose.orientation = Quaternion(*quat)        
        self.odom_pub.publish(self.odom)
        
        if not self.published_gazebo_odom:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom" 
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = 0
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.br.sendTransform(t); 
        
        self.vx_ant = vxw
        self.vy_ant = vyw
        self.px_ant = px
        self.py_ant = py
        self.theta_ant = theta
        
if __name__ == "__main__":
    try:
        node = Compute_Vel_Pose()
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        sys.exit()
