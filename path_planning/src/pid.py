#! /usr/bin/env python3
import math

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
# from f1tenth_simulator.msg import PIDInput
from geometry_msgs.msg import Twist  

#For car location and position
from nav_msgs.msg import Odometry  

class Pid:

    def __init__(self):
                
        self.pub = rospy.Publisher('nav', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("steering_angle", Twist, self.control)
        rospy.Subscriber('robot_control/odom', Odometry, self.odometry_callback)

        self.kp = 10.
        self.kd = 0.01
        self.kp_vel = 0.3 # 42
        self.kd_vel = 0.0 #0.0
        self.max_velocity = 12.0 # Ref velocity
        self.ki = 0.1   #0.0
        self.ki_vel = 0.0 #0.1 
        self.servo_offset = 18.0*math.pi/180
        self.prev_error = 0.0
        # self.error = 0.0
        self.integral = 0.0
        # self.vel_input = 1.0
        self.speed_reducer = 3 # reduces speed by this factor
        self.linear_vel = 0 # current vel
        self.min_vel = 1.7

    def control(self, data):
        # global velocity2
        #just for reference
        velocity1 = self.linear_vel
        velocity2 = self.linear_vel 

        angle = self.servo_offset
        error = abs(self.getheadingerror(data, self.max_velocity, self.speed_reducer))#5*data.pid_error # change to cte + heading ------------------------------------------------------------------------------
        # error = math.tan(data.angular.z)
        target_vel = self.max_velocity - error
        if target_vel < self.min_vel:
            target_vel = self.min_vel
        print("Reduced Heading Error", error)
        print("Target_vel", target_vel)

        vel_error = target_vel - self.linear_vel    
        if vel_error != 0.0:
            # if abs(error - prev_error)>0.5:
            #     integral = integral + error
            self.integral = self.integral + vel_error # subject to changes ----------------------------------------------------------------------------
            # # integral = integral/1.3
            control_error_vel = self.kp_vel*vel_error + self.kd_vel*(vel_error - self.prev_error) + self.ki_vel*self.integral
            print ("Control error velocity",control_error_vel)
            velocity1 = velocity1 - (control_error_vel) # /10 ------------------------------------------------------------------------
            velocity2 = velocity2 + (control_error_vel) #-----------------------------------------------------------------------------
            print("Actual velocity", self.linear_vel)
            print("Velocity substracted", velocity1)
            print("Velocity added", velocity2)
            print("Vel carla", control_error_vel)
        
        self.prev_error = control_error_vel

        # may be required(turns) --------------------------------------------------------------------------------------------------------
        '''
        if angle >= 20*np.pi/180 or angle <= -20*np.pi/180:
            velocity = 0.8
        if angle > 30*np.pi/180 or angle < -30*np.pi/180:
            velocity = 0.15
        if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
            velocity = 2.0
        if velocity < 0:
            velocity = 1
        if velocity > 2.5:
            velocity = 2.5
        prev_angle = angle
        '''
        
        # print("Velocity", velocity)
        # print("Angle", angle)
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'cmd_vel'
        msg.drive.speed = velocity2
        # msg.drive.steering_angle = angle
        self.pub.publish(msg)

    def odometry_callback(self, odometry):

        # self.carPosX = odometry.pose.pose.position.x
        # self.carPosY = odometry.pose.pose.position.y
        # orientation_q = odometry.pose.pose.orientation
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (self.roll, self.pitch, self.carPosYaw)  = euler_from_quaternion(orientation_list)                 
        self.linear_vel = np.sqrt((odometry.twist.twist.linear.x)**2 + (odometry.twist.twist.linear.y)**2)

    # def listener():
    #     rospy.init_node('pid_controller', anonymous=True)
    #     rospy.Subscriber("steering_angle", Twist, control)
    #     rospy.Subscriber('robot_control/odom', Odometry, self.odometry_callback)
    #     global kp
    #     global ki
    #     global kd
    #     global vel_input
    #     # kp = input("Enter Kp Value: ")
    #     # ki = input("Enter Ki Value: ")
    #     # kd = input("Enter Kd Value: ")
    #     # vel_input = input("Enter Velocity: ")
    #     rospy.spin()

    def getheadingerror(self, data, velocity, speed_reducer, L=3):
        
        # Adding cross track error may give better results or may not -------------------------------------------------------------------
        delta = data.angular.z
        heading_error = (velocity * math.sin(delta))/L    
        return heading_error * speed_reducer

# if __name__ == '__main__':
#     print("Listening to error for PID")
#     listener()
