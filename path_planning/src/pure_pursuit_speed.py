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
from ackermann_msgs.msg import AckermannDriveStamped

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
        self.lookahead_dist = 5.0 #5 # lookahead distance
        self.K = 0  # figure out krna hia 
        self.path_points = []
        self.world_frame = "odom"
        self.dot_product = 0
        
        # rospy.Subscriber('path_segment', Path, self.point_of_track)
        rospy.Subscriber('path_segment', Path, self.angle_pred)
        rospy.Subscriber('nav', AckermannDriveStamped, self.vel_callback)
        # rospy.Subscriber('path_segment', PoseStamped, self.angle_pred)
        rospy.Subscriber('robot_control/odom', Odometry, self.odometry_callback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        self.steeringpub = rospy.Publisher("steering_angle", Twist, queue_size = 1)
        self.waypointsVisualPub = rospy.Publisher("/visual/waypoints", MarkerArray, queue_size=1)
        self.filteredBranchVisualPub = rospy.Publisher("/visual/filtered_tree_branch", Marker, queue_size=1)
   
    def odometry_callback(self, odometry):


        headingVector = self.getHeadingVector()
        # print("headingVector:", headingVector)
        
        #print(self.carPosYaw)

        headingVectorOrt = [-headingVector[1], headingVector[0]] # -sin(yaw) and cos(yaw)
        # print("headingVectorOrt:", headingVectorOrt)

        behindDist = 1.5
        # carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]

        self.carPosX = odometry.pose.pose.position.x
        self.carPosY = odometry.pose.pose.position.y
        orientation_q = odometry.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.carPosYaw)  = euler_from_quaternion(orientation_list)                 
        
        # ###########-------------------May use it when vel_callback is not there----------------###########
        # self.linear_vel = 2.0 #np.sqrt((odometry.twist.twist.linear.x)**2 + (odometry.twist.twist.linear.y)**2)  
        
        # DOne to take rear point as reference point
        self.carPosX = self.carPosX - behindDist * headingVector[0]
        self.carPosY = self.carPosY - behindDist * headingVector[1]
    # def point_of_track(self, path):
        
    #     self.path_points = path.poses
    #     print(self.path_points)

    def vel_callback(self, vel):
        self.linear_vel = vel.drive.speed
        print("Current vel", self.linear_vel)    
    
    def angle_pred(self, pathss):

        if self.linear_vel == 0.0:
            self.linear_vel = 0.1
            # use return when actual planning is used
            #return
                
        # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (24.0, 3.0), (28.0, 5.5), (32.0, 2.8), (34.0, 2.0)] # (20.0, 1.0),
        # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.7), (17.0, 0.8), (18.0, 1.3), (21.0, 2.0), (24.0, 3.5), (28.0, 6.0), (32.0, 2.8), (34.0, 2.0)] # (20.0, 1.0),
        path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0), (36.0, 1.5), (38.0, 0.5), (38.0, -0.5), (34.0, -1.0), (32.0, -1.5), (30.0, -2.0), (27.0, -2.25), (26.0, -3.0), (22.0, -4.0), (18.0, -4.5), (16.0, -6.0), (14.0, -6.5), (10.0, -6.5), (6.0, -6.0), (3.0, -7.0), (-1.0, -7.5), (-4.0, -7.0),(-8.0, -6.5), (-10.0, -5.5), (-12.0, -5.0),(-13.0, -4.0), (-12.0, -3.0), (-10.0, -2.5),(-8.0, -1.5), (-4.0, -1.0), (-1.0, -0.5), (2.0, 0.0)]

        # To append the first 10 point of the list. 
        # When it is re iterated it should not be appended again to the list
        # if i < 13:
            # self.path_points.append((self.targetX, self.targetY))
            # i += 1
        self.path_points = path       

        # Finding out the look ahead point at a fixed dostance away from the car  
        while len(self.path_points) > 0:
            flag =1

            headingVector = self.getHeadingVector()
            # print("headingVector:", headingVector)

            #print(self.carPosYaw)

            # We getting the same vector as written below(on right)
            headingVectorOrt = [headingVector[0], headingVector[1]] # cos(yaw) and sin(yaw)
            print("headingVectorOrt:", headingVectorOrt)
            # Direction vector is list        

            # dot product between the heading vector and x axis
            # It is done to figure out if car has moved ahead of target point
            x_axis = [1.0, 0.0]
            self.dot_product = np.dot(headingVectorOrt, x_axis)

            interX, interY = self.path_points[0]
            # list of tuple containing the point of our track
                        
            if self.dist(interX, self.carPosX, interY, self.carPosY) > self.lookahead_dist:
                flag = 0
                interX = (prev_X + interX)/2
                interY = (prev_Y + interY)/2

            print(self.path_points)              
            if interX*self.dot_product > self.carPosX*self.dot_product:
                
                if flag != 0:
                    ind = self.path_points.index((interX, interY))
                else:
                    ind = -1
                    # flag = 1
                print("index of next target", ind)
                #use ind and ind+1 to make pints between them
                
                nextinterX, nextinterY = self.path_points[ind + 1]
                a = (interX**2) - (2*self.carPosX*interX) + (interY*interY) - (2*self.carPosY*interY) - (self.lookahead_dist**2) + (self.carPosY*self.carPosY) + (self.carPosX*self.carPosX)
                b = (2*interX*nextinterX) - (2*self.carPosX*interX) - (2*self.carPosX*nextinterX) + (2*nextinterY*interY) - (2*self.carPosY*interY) -(2*self.carPosY*nextinterY) + (2*self.carPosY*self.carPosY) + (2*self.carPosX*self.carPosX) - (2*(self.lookahead_dist**2))
                c = (nextinterX*nextinterX) + (nextinterY*nextinterY) + (self.carPosY*self.carPosY) + (self.carPosX*self.carPosX) - (2*self.carPosX*nextinterX) - (2*self.carPosY*nextinterY) - (self.lookahead_dist**2)
                
                m_root = self.equationroots(a, b, c)

                if (m_root == False) and ((interX, interY) in self.path_points):
                    print("----------------------------------------------REMOVED Bcoz m_ratio is negative. This means the given look ahead point has crossed the nearest waypoint------------------------------------------------")
                    prev_X, prev_Y = interX, interY
                    self.path_points.remove((interX, interY))
                
                print("interX", interX, "interY", interY)
                print("nextinterX", nextinterX, "nextinterY", nextinterY)                
                self.target_pointX = (nextinterX + (m_root*interX))/(1 + m_root)
                self.target_pointY = (nextinterY + (m_root*interY))/(1 + m_root)

                self.publishlookaheadpointsVisuals(self.target_pointX, self.target_pointY)
                self.publishFilteredBranchVisual(path)

                # self.target_pointX, self.target_pointY = self.get_lookahead_dist(self.path_points)
                self.ld = self.dist(self.target_pointX, self.carPosX, self.target_pointY, self.carPosY)
                self.K = 6.0 #self.ld/self.linear_vel
                
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
                if alpha >= 1.309 or self.steering_angle >= 1.309:
                    self.steering_angle = 1.309
                elif alpha <= -1.309 or self.steering_angle <= -1.309:
                    self.steering_angle = -1.309
                print(self.steering_angle)

                # if distance between the car and the target point is less than 50 cm 
                # then delete the this point from the list
                if (self.dist(nextinterX, self.carPosX, nextinterY, self.carPosY) < self.lookahead_dist) and ((interX, interY) in self.path_points):

                # if (self.target_pointX*self.dot_product < self.carPosX*self.dot_product) or (nextinterX*self.dot_product < self.carPosX*self.dot_product):
                    print("---------------------------------------------------------------REMOVED bcoz the target point is closer than the lookahead distance-------------------------------------------------------------")
                    prev_X, prev_Y = interX, interY
                    # del self.path_points[0]
                    self.path_points.remove((interX, interY))    


                # publish the waypoints of the track
                # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (20.0, 1.0), (21.0, 1.5), (26.0, 3.0), (30.0, 2.8), (32.0, 2.0)]
                # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.7), (17.0, 0.8), (18.0, 1.3), (21.0, 2.0), (24.0, 3.5), (28.0, 6.0), (32.0, 2.8), (34.0, 2.0)] # (20.0, 1.0),
                # path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0), (36.0, 1.5), (38.0, 0.5), (38.0, -0.5), (34.0, -1.0), (32.0, -1.5), (30.0, -2.0), (27.0, -2.25), (26.0, -3.0), (22.0, -4.0), (18.0, -3.5), (16.0, -2.0), (14.0, -1.5), (10.0, -1.5), (8.0, -2.0), (6.0, -2.5), (2.0, -2.0), (0.0, -1.5), (0.0, -1.0), (2.0, 0.0)]
                path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0), (36.0, 1.5), (38.0, 0.5), (38.0, -0.5), (34.0, -1.0), (32.0, -1.5), (30.0, -2.0), (27.0, -2.25), (26.0, -3.0), (22.0, -4.0), (18.0, -4.5), (16.0, -6.0), (14.0, -6.5), (10.0, -6.5), (6.0, -6.0), (3.0, -7.0), (-1.0, -7.5), (-4.0, -7.0),(-8.0, -6.5), (-10.0, -5.5), (-12.0, -5.0),(-13.0, -4.0), (-12.0, -3.0), (-10.0, -2.5),(-8.0, -1.5), (-4.0, -1.0), (-1.0, -0.5), (2.0, 0.0)]
                
                self.publishWaypointsVisuals(path)
            

                # Now that we predicted our angle. It is time to publish them
                msg = Twist()

                #Publishing the linear velocity and steering angle values
                msg.linear.x = self.linear_vel/math.sqrt(2)
                msg.linear.y = self.linear_vel/math.sqrt(2)
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = self.steering_angle
                
                self.pub.publish(msg)
                self.steeringpub.publish(msg)

            else:
                print("--------------------------------------------------------------REMOVED Bcoz our car has crossed it--------------------------------------------------------")
                prev_X, prev_Y = interX, interY
                if ((interX, interY) in self.path_points):
                    self.path_points.remove((interX, interY))    

            # if len(self.path_points) == 2:
            #     self.linear_vel = 1
            # if len(self.path_points) == 1:
            #     self.linear_vel = 0.3
            # if len(self.path_points) == 0:
            #     self.linear_vel = 0.0     
                """THink about the end wherein the car is about to reach the goal point. 
                   Linear, angular velocity and lookahead mechanism may change to what was done previously.
                   Also see how multiple path may be integrated to it when published from path planning.
                   for later half you can use what you did in point cloud process file to send list of cones 
                   and unpacked it in RRTstar.py. Same list can be used in here. """                               


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

     # To publish look ahead points of the track
    def publishlookaheadpointsVisuals(self, target_pointX, target_pointY):

        markerArray = MarkerArray()

        if target_pointX is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = rospy.Time.now()
            newWaypointsMarker.lifetime = rospy.Duration(1)
            newWaypointsMarker.ns = "new-publishlookaheadpointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1
            newWaypointsMarker.scale.x = 0.3
            newWaypointsMarker.scale.y = 0.3
            newWaypointsMarker.scale.z = 0.3

            newWaypointsMarker.color.a = 0.65
            newWaypointsMarker.color.r = 1.0

            # for waypoint in newWaypoints:
            p = Point(target_pointX, target_pointY, 0.0)
            newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)


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

        # function for finding roots 

    def equationroots(self, a, b, c):  
        
        # calculating discriminant using formula 
        dis = b * b - 4 * a * c  
        sqrt_val = math.sqrt(abs(dis))  

        if dis > 0:
            
            print(" real and different roots ")  
            print("root1", (-b + sqrt_val)/(2 * a))  
            print("root2", (-b - sqrt_val)/(2 * a))
            root1 = (-b + sqrt_val)/(2 * a)
            root2 = (-b - sqrt_val)/(2 * a)  
            if (root1 > 0) and (root2 <0):
                return root1
            elif (root2 > 0) and (root1 < 0):
                return root2
            else:
                return False    
        
        elif dis == 0:  

            print(" real and same roots")  
            print("root1", -b / (2 * a))
            root1 = -b / (2 * a)
            if (root1 > 0):
                return root1
            else:
                return False

        # when discriminant is less than 0 
        else:

            print("Complex Roots")  
            print("root1", - b / (2 * a), " + i", sqrt_val)  
            print("root2", - b / (2 * a), " - i", sqrt_val)
            return False  
        
    def publishFilteredBranchVisual(self, path):

        if not path:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(0.2)
        marker.ns = "publisshFilteredBranchVisual"

        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07

        marker.pose.orientation.w = 1

        marker.color.a = 0.7
        marker.color.b = 1.0

        for i in range(len(path)):
            node = path[i]
            p = Point(node[0], node[1], 0)
            if i != 0:
                marker.points.append(p)

            if i != len(path) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(marker)