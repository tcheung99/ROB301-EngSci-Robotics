#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#write a class 
class Camera:
	def __init__(self):
		self.data = -1
		return None

	def callback(self,data):
	 #   rospy.loginfo((data))
		self.data = data.data
		#print data

	def publisher_node(self):
		cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		cmd_sub = rospy.Subscriber('color_mono', String, self.callback, queue_size=1)

		rospy.sleep(2)
		twist=Twist()
		init_t = rospy.get_time()
		rate = rospy.Rate(10)
			
		desired = 320 
		#k_p = 0.0029		
		#k_i = 0.0085
		k_p = 0.0055		
		#k_i = 0.0006		
		k_i = 0.00085
		integral = 0
		while rospy.get_time()-init_t < (27):
			actual = int(self.data)
			print actual
			error = desired - actual 
			if ((integral+error) < 120 ) and ((integral+error) > - 120):
				integral = integral + error 

			print integral
			correction = k_p*error + k_i*integral
			twist.linear.x = 0.1 
			#print correction
			twist.angular.z = correction
			cmd_pub.publish(twist)
			rate.sleep()
			#rospy.loginfo(twist.angular.z
		twist.linear.x = 0
		twist.angular.z = 0
		rospy.loginfo(twist.linear.x)
		rospy.loginfo(twist.angular.z)

		print "HELLO"
		cmd_pub.publish(twist)
		pass


def main():
    data = 0
    try:
        rospy.init_node('motor')
	C = Camera()
        C.publisher_node()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    main()
