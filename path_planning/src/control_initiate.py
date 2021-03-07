#! /usr/bin/env python3

import rospy
# from pure_pursuit import Pure_pursuit
from pure_pursuit_planner import Pure_pursuit

def main():
    rospy.init_node("pure_pursuit", anonymous= True)
    lat_control = Pure_pursuit()
    # lat_control.angle_pred()
    rospy.spin()

''' 
    rate = rospy.Rate(1000) # big amount on purpose

    while not rospy.is_shutdown():
        initiate.processing()
        rate.sleep()
'''
    # Spin until ctrl + c
    

if __name__ == '__main__':
    main()