#! /usr/bin/env python3

import math
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
#i=0
#path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0), (13.0, 0.0), (16.0, 0.8), (20.0, 1.0)]
def Trajectory():
    # Real and the next one is for visualization
    #path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (20.0, 1.0), (21.0, 1.5), (26.0, 3.0), (30.0, 2.8), (32.0, 2.0)]
    path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0), (36.0, 1.5), (38.0, 0.5), (38.0, -0.5), (34.0, -1.0), (32.0, -1.5), (30.0, -2.0), (27.0, -2.25), (26.0, -3.0), (22.0, -4.0), (18.0, -3.5), (16.0, -2.0), (14.0, -1.5), (10.0, -1.5), (8.0, -2.0), (6.0, -2.5), (2.0, -2.0), (0.0, -1.5), (0.0, -1.0), (2.0, 0.0)]# (20.0, 1.0),

    #global i 
    pitch=0
    roll=0
    carPosX, carPosY = (0.0, 0.0)
    track = Path()
    track.header.frame_id = "base_link"
    track.header.stamp = rospy.Time.now()
    for point_X, point_Y in path:
        #yaw = math.atan((point_Y - carPosY)/(point_X - carPosX))
        #print(yaw)
        #point_X,point_Y = path[i]  
        dir_vec = np.array([point_X - carPosX, point_Y - carPosY])
        axis = np.array([1.0, 0.0])
            #do dot product between them
        cos_theta = np.dot(dir_vec, axis)/(np.sqrt((point_X - carPosX)**2 + (point_Y - carPosY)**2))
        yaw = math.acos(cos_theta)
            
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            
        posess = PoseStamped()
        # track.poses.append(PoseStamped((pose.position.x = point_X),
            #                                (pose.position.y = point_Y),
            #                                (pose.position.z = 0),
                                        #    (pose.orientation.x = qx),
                                        #    (pose.orientation.y = qy),
                                        #    (pose.orientation.z = qz),
                                        #    (pose.orientation.w = qw)))
                                        
        posess.pose.position.x = point_X
        posess.pose.position.y = point_Y
        posess.pose.position.z = 0.0
        posess.pose.orientation.x = qx
        posess.pose.orientation.y = qy
        posess.pose.orientation.z = qz
        posess.pose.orientation.w = qw
        track.poses.append(posess)

        #print(track.poses)
        #print(posess)

        pub.publish(track)  # This is when pure_pursuit.py file is being used 
        #pub.publish(posess)  # This is when pure_pursuit_planner.py file is being used

        carPosX, carPosY = point_X, point_Y
        
        # i += 1
        # if i >= 13:
        #     i = 0

# def Path():
#     #path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0), (13.0, 0.0), (16.0, 1.5), (20.0, 2.5)]
#     global i
#     point_X, point_Y = path[i]
#     i += 1
    
#     if i==8:
#        i = 0

#     pitch=0
#     roll=0
#     carPosX, carPosY = (0.0, 0.0)
#     track = Path()
#     track.header.frame_id = "base_link"
#     track.header.stamp = rospy.Time.now()
#     #for point_X, point_Y in path:
#     #yaw = math.atan((point_Y - carPosY)/(point_X - carPosX))
#     #print(yaw)  
    
#     dir_vec = np.array([point_X - carPosX, point_Y - carPosY])
#     axis = np.array([1.0, 0.0])
 
# # #do dot product between them
#     cos_theta = np.dot(dir_vec, axis)/(np.sqrt((point_X - carPosX)**2 + (point_Y - carPosY)**2))
#     yaw = math.acos(_cos_theta)
    
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
#     pose = PoseStamped()
#         # track.poses.append(PoseStamped((pose.position.x = point_X),
#         #                                (pose.position.y = point_Y),
#         #                                (pose.position.z = 0),
#                                     #    (pose.orientation.x = qx),
#                                     #    (pose.orientation.y = qy),
#                                     #    (pose.orientation.z = qz),
#                                     #    (pose.orientation.w = qw)))
                                       
#     pose.pose.position.x = point_X
#     pose.pose.position.y = point_Y
#     pose.pose.position.z = 0.0
#     pose.pose.orientation.x = qx
#     pose.pose.orientation.y = qy
#     pose.pose.orientation.z = qz
#     pose.pose.orientation.w = qw
#     track.poses.append(pose)
        
#     pub.publish(track)

#     carPosX, carPosY = point_X, point_Y

# def interm(k=0):
#     path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.0), (13.0, 0.0), (16.0, 1.5), (20.0, 2.5)]
#     point_X, point_Y = path[k]
#     path(point_X, point_Y)

#     #k = path.index((point_X, point_Y))
#     print(k)
#     if k < 8:
#         k += 1
#     else:
#         k = 0    

if __name__ == "__main__":
    rospy.init_node("path_for_control", anonymous= True)
    pub = rospy.Publisher('path_segment', Path, queue_size = 1)
    #pub = rospy.Publisher('path_segment', PoseStamped, queue_size = 1)
    #k = 0
    rate = rospy.Rate(0.2) # small amount on purpose (20m in 20 sec)

    while not rospy.is_shutdown():
        Trajectory()
        #Path()    
        #interm()
        rate.sleep()    
    # rospy.spin()