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
    current_time = 0
    i=0
#Xa
    for i in range (0,3):
	current_time = rospy.get_time()-init_2
    	while current_time < (10): #check while loops; add delays to avoid motor problems
		current_time = rospy.get_time()-init_t
	    	#rospy.sleep(1) 
		twist.linear.x=0.1
		cmd_pub.publish(twist)
		rospy.loginfo(twist.linear.x)
		rate.sleep()
	init_2 = rospy.get_time()
	current_time = rospy.get_time()-init_2
	while current_time < ((math.pi)/.4):      #90 deg
		#while rospy.get_time()-init_2 < (2.5):  
		current_time = rospy.get_time()-init_2
		twist.linear.x = 0
		twist.angular.z = 0.2
		cmd_pub.publish(twist)
		rate.sleep()
		#rospy.loginfo(twist.angular.z

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
