#! /usr/bin/env python3

import rospy
# from pure_pursuit import Pure_pursuit
from pid import Pid

def main():
    rospy.init_node('pid_controller', anonymous=True)
    long_control = Pid()
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