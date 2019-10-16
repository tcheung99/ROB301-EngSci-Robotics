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
			
		desired = 350 
		#k_p = 0.0029		
		#k_i = 0.0085
		k_p = 0.0058		
		#k_i = 0.0006		
		#k_i = 0.00185				
		k_i = 0.00196		
		#k_i = 0.00095
		integral = 0
		derivative = 0
		lasterror = 0
		#k_d = 0.0007		
		#k_d = 0.00095
		
		#k_d = 0.00098
		k_d = 0.00105
		while rospy.get_time()-init_t < (16):
			actual = int(self.data)
			print actual
			error = desired - actual 
			if ((integral) < 100 ) and ((integral) > -100):
				integral = integral + error 

			print integral
			derivative = error - lasterror
			correction = k_p*error + k_i*integral+ k_d*derivative
			twist.linear.x = 0.08
			#print correction
			twist.angular.z = correction
			cmd_pub.publish(twist)
			lasterror = error
			rate.sleep()
			#rospy.loginfo(twist.angular.z
		twist.linear.x = 0
		twist.angular.z = 0
		rospy.loginfo(twist.linear.x)
		rospy.loginfo(twist.angular.z)

		#print "HELLO"
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
