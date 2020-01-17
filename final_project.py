#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class BayesLoc:

    def __init__(self, color_map):
        self.colour_sub = rospy.Subscriber('mean_img_rgb', String, self.measurement_callback)
        self.line_idx_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_map = color_map
        self.measured_rgb = np.array([0,0,0]) # updated with the measurement_callback
        self.line_idx = 0 # updated with the line_callback with the index of the detected black line.

    def measurement_callback(self, msg):
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.measured_rgb = np.array([r,g,b])
        
    def line_callback(self, data):
        index = int(data.data)
        self.line_idx = index

    def publisher_node(self):
		rospy.sleep(2)
		twist=Twist()
		init_t = rospy.get_time()
		rate = rospy.Rate(10)
		#Line-following Controller Initialization
		desired = 300 # Line index value at which robot is centred 
		k_p = 0.0057				
		correction = 0

		#Bayesian Localization Parameter Initialization 
		state_model = [0.05, 0.1, 0.85] # State model for u_k = +1
		meas_model = {0:{0:0.6,1:0.05,2:0.2,3:0.05}, #green
					1:{0:0.05,1:0.6,2:0.05,3:0.15}, #orange
					2:{0:0.2,1:0.05,2:0.6,3:0.05},  #blue 
					3:{0:0.05,1:0.2,2:0.05,3:0.65}, #yellow
					4:{0:0.1,1:0.1,2:0.1,3:0.1}}	#line 
		#Measurement_model represented as dictionary with keys as measurement z_k and value being a dictionary with key x_k and corresponding probabilities
		pred = np.zeros(12) # Position prediction at each office location
		update = np.zeros(12,dtype=np.float) # Update to state estimation probabilities based on prediction and measurement model 
		norm = np.zeros(12) # Normalization constant at each update to the state estimation
		curr_state = [1/float(len(pred))]*len(pred) # Current state estimation, starting with uniform probability distribution 

		# Reference RGB colour codes 
		ref = np.zeros((5,3))
		ref[0] = [133,163,1] #green
		ref[1] = [222,48,1] #orange
		ref[2] = [35,60,82] #blue
		ref[3] = [171,140,2] #yellow
		ref[4] = [120,96,34] #line
			
		j = 0 
		while rospy.get_time()-init_t < (250):			
			actual = int(self.line_idx)
			colour = None 	

			rgb_error = np.abs(ref - self.measured_rgb)
			rgb_sum = np.sum(rgb_error, axis=1)
			min_ind = np.argmin(rgb_sum) # Colour estimation

			if ((min_ind == 0) or (min_ind == 1) or (min_ind == 2) or (min_ind == 3)): # If a colour is seen 
				correction = 0
				colour = min_ind 
			elif (min_ind == 4): # If no colour measured/seen (detected a line)
				error = desired - actual  
				correction = k_p*error # Correction for P control 

			twist.angular.z = correction
			twist.linear.x = 0.05
			self.cmd_pub.publish(twist)

			curr_office = np.argmax(curr_state) + 1 # Current state prediction 
			print(curr_state, (curr_office))

			if correction == 0 and colour!=None: # If a colour is seen 
				if curr_office == 4 or curr_office ==6 or curr_office ==8: # If we are at our chosen office location for delivery					
					init_2 = rospy.get_time()
					rospy.sleep(3)				
					while rospy.get_time()-init_2 < ((math.pi)/.32):      #90 degree left rotation
						twist.linear.x = 0
						twist.angular.z = 0.2
						self.cmd_pub.publish(twist)
						rate.sleep()
					init_3 = rospy.get_time()
					rospy.sleep(1) # Pause
					while rospy.get_time()-init_3 < ((math.pi)/.48):      #90 degree right rotation
						twist.linear.x = 0
						twist.angular.z = -0.2
						self.cmd_pub.publish(twist)
						rate.sleep()
					# Resume straight-line motion 
					twist.linear.x = 0.05
					twist.angular.z = 0	
					self.cmd_pub.publish(twist)
					rospy.sleep(3)
				else:
					rospy.sleep(6.69) # Continue straight-line motion 
				for i in range(0,len(curr_state)): # Position prediction step
					if (i==0): # First point in map 
						step_fwd = curr_state[len(curr_state)-1]*state_model[2]
					if (i!=0):
						step_fwd = curr_state[i-1]*state_model[2] #loop around 
					if (i==len(curr_state)-1):
						step_bwd = curr_state[0]*state_model[0]
					if (i!=len(curr_state)-1):
						step_bwd = curr_state[i+1]*state_model[0]
					step_none = curr_state[i]*state_model[1] 
					pred[i] = (step_fwd+step_bwd+step_none)
				for k in range(0,len(pred)): #update based on measurement
					update[k] = pred[k]*meas_model[colour][color_map[k]]
					norm[j] += update[k]
				update = [float(x)/float(norm[j]) for x in update]
				curr_state = update # This evolves at each colour measurement
				j += 1 # Increment j every time you get a measurement 		
			rate.sleep()	
		pass

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    color_map = [0,1,0,3,2,0,1,3,2,2,1,3] # Map of mail delivery simulation environment 
                 
    rospy.init_node('bayes_loc')
    BL=BayesLoc(color_map)
    rospy.sleep(0.5)

    try:
        BL.publisher_node()
        while (1):
            key = getKey()
            if (key == '\x03'): #1.22:bayesian.curPos >= 1.6 or
                rospy.loginfo('Finished!')
                break
            rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
            rospy.loginfo("Line index: {}".format(BL.line_idx))
                
    except Exception as e:
        print("comm failed:{}".format(e))

    finally:
        ### Stop the robot when code ends
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_publisher.publish(twist)
