#! /usr/bin/env python3

'''
author Kartikey vishnu
       Kartikey-ai(TDR-SDC)
'''

import rospy
import math
import numpy as np
from nav_msgs.msg import Path

#For car location and position
from nav_msgs.msg import Odometry  

# Path points for navigation
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist  

# For odometry message
from tf.transformations import euler_from_quaternion

# for visualizations
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

i=0

class Pure_pursuit:

    def __init__(self):
        self.carPosX = 0
        self.carPosY = 0
        self.target_pointX = 0
        self.target_pointY = 0
        self.roll, self.pitch, self.carPosYaw = (0.0, 0.0, 0.0)
        self.linear_vel = 0
        self.ld = 0
        self.K = 0  # figure out krna hia 
        self.path_points = []
        self.world_frame = "odom"
        
        # rospy.Subscriber('path_segment', Path, self.point_of_track)
        rospy.Subscriber('path_segment', PoseStamped, self.angle_pred)
        rospy.Subscriber('robot_control/odom', Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)

        

    def odometry_callback(self, odometry):

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        orientation_q = odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.carPosYaw)  = euler_from_quaternion(orientation_list)                 
        self.linear_vel = 2.0 #np.sqrt((odometry.twist.twist.linear.x)**2 + (odometry.twist.twist.linear.y)**2)

    # def point_of_track(self, path):
        
    #     self.path_points = path.poses
    #     print(self.path_points)
            
    
    def angle_pred(self, points):
        
        global i 
        # self.target_pointX = self.path_points.pose.position.x
        # self.target_pointY = self.path_points.pose.position.y
        self.targetX = points.pose.position.x
        self.targetY = points.pose.position.y
        
        # To append the first 10 point of the list. 
        # When it is re iterated it should not be appended again to the list
        if i < 13:
            self.path_points.append((self.targetX, self.targetY))
            i += 1

        # list of tuple containing the point of our track
        print(self.path_points)
        self.target_pointX, self.target_pointY = self.path_points[0]
        self.ld = self.dist(self.target_pointX, self.carPosX, self.target_pointY, self.carPosY)
        self.K = 6.0 #self.ld/self.linear_vel
        
        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)
        
        #print(self.carPosYaw)

        # We getting the same vector as written below(on right)
        headingVectorOrt = [headingVector[0], headingVector[1]] # cos(yaw) and sin(yaw)
        print("headingVectorOrt:", headingVectorOrt)
        # Direction vector is list
        
        #np.sqrt((self.target_pointX - self.carPosX)**2 + (self.target_pointY - self.carPosY)**2)
        dir_vec_mag = self.dist(self.target_pointX, self.carPosX, self.target_pointY, self.carPosY)
        dir_vec = [(self.target_pointX - self.carPosX)/dir_vec_mag, (self.target_pointY - self.carPosY)/dir_vec_mag]
        print("dir vec is", dir_vec)
        print("carpoint_X", self.carPosX)
        print('targetpoint_X', self.target_pointX)
        print("carpointY", self.carPosY)
        print("targetpoint_Y", self.target_pointY)

        # alpha we are getting is the angle between the car heading and the vector (between target point and ref point) 
        #print(np.dot(dir_vec, headingVectorOrt))
        alpha = math.acos(np.dot(dir_vec, headingVectorOrt))
        
        # If sin component of heading of car is less than 
        # sin component of dir_vec than angle is towards left otherwise right 
        if headingVectorOrt[1] > dir_vec[1]:
            alpha  = (-1)*alpha         
        
        print("alpha is", alpha)
        print("sin(alpha) is", math.sin(alpha)) 

        self.steering_angle = math.atan(6*math.sin(alpha)/(self.K* self.linear_vel))
        
        # Max steering angle limits for aplha greater than 90 deg.
        if alpha >= 1.05 or self.steering_angle >= 1.05:
            self.steering_angle = 1.05
        elif alpha <= -1.05 or self.steering_angle <= -1.05:
            self.steering_angle = -1.05
        print(self.steering_angle)

        # dot product between the heading vector and x axis
        # It is done to figure out if car has moved ahead of target point
        x_axis = [1.0, 0.0]
        dot_product = np.dot(headingVectorOrt, x_axis)

        # if distance between the car and the target point is less than 50 cm 
        # then delete the this point from the list
        if (self.dist(self.target_pointX, self.carPosX, self.target_pointY, self.carPosY) < 50e-2) or (self.target_pointX*dot_product < self.carPosX*dot_product):
            del self.path_points[0]

        # publish the waypoints of the track
        # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (20.0, 1.0), (21.0, 1.5), (26.0, 3.0), (30.0, 2.8), (32.0, 2.0)]
        path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0)] # (20.0, 1.0),
        self.publishWaypointsVisuals(path)
    

        # Now that we predicted our angle. It is time to publish them
        msg = Twist()

        #Publishing the linear velocity and steering angle values
        msg.linear.x = 2.0/math.sqrt(2)
        msg.linear.y = 2.0/math.sqrt(2)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.steering_angle
        
        self.pub.publish(msg)


    def getHeadingVector(self):
        #self.carPosYaw = 3.14/6
        # Need to check whethe rthe heading vector is list or needs to be an array?????????????????????????????
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    # Distance between two points given x1, x2, y2, y1 values
    def dist(self, x1, x2, y1, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq


     # To publish way points of the track
    def publishWaypointsVisuals(self, newWaypoints):

        markerArray = MarkerArray()

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = Point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)    
