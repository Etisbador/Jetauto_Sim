#!/usr/bin/env python3

import os
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray, Header
from jetauto_interfaces.msg import imu_encoder
from rosgraph_msgs.msg import Clock

class WheelSpeedController:
    def __init__(self):
        rospy.init_node("wheel_speed_controller", anonymous=False)
        
        # Parameters for the PID controller
        self.Kp = rospy.get_param("~Kp", 1.95)
        self.Ki = rospy.get_param("~Ki", 4.8)
        self.Kd = rospy.get_param("~Kd", 0.00)
        
        self.e_ant = [0,0,0,0]
        self.control_outputs_ant = [0, 0, 0, 0]
        
        # Setpoint for the wheel speed
        self.setpoint = rospy.get_param("~setpoint", [0.0, 0.0, 0.0, 0.0])  # Example setpoints for four wheels
        
        #Calculo constantes para el controlador
        self.Ts = 1.0/90.909       
        
        
        self.A0 = self.Kp + self.Ki * self.Ts / 2 + 2 * self.Kd / self.Ts
        self.A1 = -self.Kp + self.Ki * self.Ts / 2 - 2 * self.Kd / self.Ts
        self.A2 = self.Kd / self.Ts  # Additional term needed for proper derivative calculation
        
        
        
        
        # Publisher for the control output
        self.control_publisher = rospy.Publisher("jetauto_wheel_cmd_sim", Float32MultiArray, queue_size=10)
        
        # Subscriber for the wheel speed
        rospy.Subscriber("/imu_encoder_sim", imu_encoder, self.control_callback)
        rospy.Subscriber("/wheel_setpoint", Float32MultiArray, self.setpoint_callback)

        
    def control_callback(self, msg):
        # Read current wheel speeds
        current_speeds = [msg.w1, msg.w2, msg.w3, msg.w4]

        # Ensure we have the correct number of wheel speeds
        if len(current_speeds) != 4:
            rospy.logwarn("Received wheel speed data does not match expected number of wheels (4).")
            return
        
        control_outputs = [0,0,0,0]
        
        # Compute control outputs for each wheel
        for i in range(4):
            error = self.setpoint[i] - current_speeds[i]
            # Compute control output with the Tustin method
            
            if self.setpoint[i] == 0:
                control_outputs[i] = 0
            else:
                control_outputs[i] = self.control_outputs_ant[i] + self.A0 * error + self.A1 * self.e_ant[i] + self.A2 * (error - self.e_ant[i])
            
            if (control_outputs[i]>=9.0):
                control_outputs[i] = 9.0
            if (control_outputs[i]<=-9.0):
                control_outputs[i] = -9.0
            
            # Update previous values
            self.control_outputs_ant[i] = control_outputs[i]
            self.e_ant[i] = error
            
        # Create a message for the control outputs
        #print(self.e_ant)
        control_msg = Float32MultiArray(data=control_outputs)

        # Publish the control outputs
        self.control_publisher.publish(control_msg)
        
    def setpoint_callback(self, msg):
        # Update the setpoints for each PID controller
        new_setpoints = msg.data

        # Ensure we have the correct number of setpoints
        if len(new_setpoints) != 4:
            #rospy.logwarn("Received setpoint data does not match expected number of wheels (4).")
            return

        self.setpoint = new_setpoints
        
    def stop(self):
        stop_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        # Publish the stop message
        self.control_publisher.publish(stop_msg)

if __name__ == "__main__":
    try:
        node = WheelSpeedController()
        rospy.spin()
        if rospy.is_shutdown():
            node.stop()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        node.stop()
        sys.exit()
