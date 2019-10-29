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
		self.meas = -1
		return None

	def callback(self,data):

		self.data = data.data
	 	#rospy.loginfo((data))
		#print ("cam, %d", data)
	
	def callback2(self,data2):
		#print(data2)
		self.state = data2.data
	 	#rospy.loginfo((data2.data))
		#print ("meas, %d", data2)

	def publisher_node(self):
		cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		cmd_sub = rospy.Subscriber('state', String, self.callback2, queue_size=1)
		cmd_sub = rospy.Subscriber('color_mono', String, self.callback, queue_size=1)
		rospy.sleep(2)
		twist=Twist()
		init_t = rospy.get_time()
		rate = rospy.Rate(10)
			
		desired = 350 
		k_p = 0.0058			
		k_i = 0.00196	
		integral = 0
		derivative = 0
		lasterror = 0
		k_d = 0.00105
		#k_d = 0.0
		counter = 1
		dict = {1: 0.61 , 2: 1.22, 3: 2.44, 4 : 3.05}

		while (counter<6):
			actual = int(self.data)
#			actual1 = float(self.meas)
			state = float(self.state)
			#if (state<=dict[counter]):
			rospy.loginfo(state) 		
			#print actual
			error = desired - actual 
			if ((integral) < 100 ) and ((integral) > -100):
				integral = integral + error 
			derivative = error - lasterror
			correction = k_p*error + k_i*integral+ k_d*derivative
			twist.linear.x = 0.05
			twist.angular.z = correction
			cmd_pub.publish(twist)
			lasterror = error
			rate.sleep()
			if (state>dict[counter]):
				#print('hey')
				twist.linear.x = 0
				twist.angular.z = 0
				rospy.loginfo(twist.linear.x)
				rospy.loginfo(twist.angular.z)
				cmd_pub.publish(twist)
				rospy.sleep(2)
				counter += 1
				if (counter==5):
					break
				
				
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
