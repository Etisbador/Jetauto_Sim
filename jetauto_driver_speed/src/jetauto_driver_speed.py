#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from simple_pid import PID
from jetauto_interfaces.msg import imu_encoder

class WheelSpeedController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('wheel_speed_controller', anonymous=True)

        # Parameters for the PID controller
        self.Kp = rospy.get_param("~Kp", 0.5)
        self.Ki = rospy.get_param("~Ki", 2.9)
        self.Kd = rospy.get_param("~Kd", 0.0)

        # Setpoint for the wheel speed
        self.setpoint = rospy.get_param("~setpoint", [0.0, 0.0, 0.0, 0.0])  # Example setpoints for four wheels

        # PID controllers for each wheel
        self.pid_controllers = [PID(self.Kp, self.Ki, self.Kd, setpoint=sp) for sp in self.setpoint]
        self.pid_controllers[0].output_limits = (-9, 9)
        self.pid_controllers[1].output_limits = (-9, 9)
        self.pid_controllers[2].output_limits = (-9, 9)
        self.pid_controllers[3].output_limits = (-9, 9)
        # Subscriber for the wheel speed
        rospy.Subscriber("/imu_encoder_sim", imu_encoder, self.control_callback)
        rospy.Subscriber("/wheel_setpoint", Float32MultiArray, self.setpoint_callback)

        # Publisher for the control output
        self.control_publisher = rospy.Publisher("jetauto_wheel_cmd_sim", Float32MultiArray, queue_size=10)
        
    def control_callback(self, msg):
        # Read current wheel speeds
        current_speeds = [msg.w1, msg.w2, msg.w3, msg.w4]

        # Ensure we have the correct number of wheel speeds
        if len(current_speeds) != 4:
            rospy.logwarn("Received wheel speed data does not match expected number of wheels (4).")
            return

        # Compute control outputs for each wheel
        control_outputs = [pid(current_speed) for pid, current_speed in zip(self.pid_controllers, current_speeds)]

        # Create a message for the control outputs
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

        for pid, sp in zip(self.pid_controllers, new_setpoints):
            pid.setpoint = sp

        #rospy.loginfo(f"Setpoints updated to: {new_setpoints}")

    def run(self):
        rospy.spin()
    
    def stop(self):
        stop_msg = Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])

        # Publish the stop message
        self.control_publisher.publish(stop_msg)

if __name__ == "__main__":
    try:
        controller = WheelSpeedController()
        controller.run()
        if rospy.is_shutdown():
            controller.stop()

    except rospy.ROSInterruptException:
        pass

