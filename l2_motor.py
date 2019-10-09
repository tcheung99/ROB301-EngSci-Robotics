#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



def publisher_node():
    print('TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.sleep(2)
    counter = 0
    counter2 = 0
    twist=Twist()
    init_t = rospy.get_time()
    rate = rospy.Rate(10)
    while rospy.get_time()-init_t < (20): #check while loops; add delays to avoid motor problems
    	#rospy.sleep(1) 
	twist.linear.x=0.1
	cmd_pub.publish(twist)
	rospy.loginfo(twist.linear.x)
        rate.sleep()
    init_2 = rospy.get_time()
    while rospy.get_time()-init_2 < ((math.pi)/.38):      #90 deg
    #while rospy.get_time()-init_2 < (2.5):  
	twist.linear.x = 0
	twist.angular.z = 0.2
	cmd_pub.publish(twist)
        rate.sleep()
	#rospy.loginfo(twist.angular.z)
    init_3 = rospy.get_time()
    while rospy.get_time()-init_3 < (5): #check while loops; add delays to avoid motor problems
    	#rospy.sleep(1) 
	twist.angular.z = 0
	twist.linear.x=0.1
	cmd_pub.publish(twist)
	rospy.loginfo(twist.linear.x)
        rate.sleep()
    init_4 = rospy.get_time()
    while rospy.get_time()-init_4 < ((math.pi)/.8):  
	twist.linear.x=0
	twist.linear.y=0
	twist.angular.z = 0.2
	cmd_pub.publish(twist)
	rospy.loginfo(twist.angular.z)
	rate.sleep()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.angular.z = 0
    cmd_pub.publish(twist)
#    rospy.loginfo(twist.angular.z)
    pass


def main():
    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    main()
