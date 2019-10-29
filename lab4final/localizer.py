#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

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

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0, S_0, W_0):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0
	self.w = 0
	self.S = S_0
	self.W = W_0

        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        self.phi_hat = np.nan
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)
	self.covariance = []
	#cmd_sub = rospy.Subscriber('scan_angle', String, self.callback, queue_size=1)
	#self.angle = 0
	self.states = []

    def covar(self):
	self.predict(self.u)
	self.D = self.h/((self.d-self.x)**2+self.h**2) 
	self.P = self.P + self.Q
	self.S = (self.D**2)*self.P + self.R
	self.W = self.P*self.D*self.S**(-1)
	self.P = self.P-(self.W**2)*self.S
	self.covariance += [self.P] 
#    def callback(self, angle):
#	self.angle = float(angle.data)
#	print("gvhk")

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*np.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self, u):

	#t_2 = rospy.get_time()
	self.x = self.u*(1/30.0) + self.x

	#print(self.u*(1/30.0))
	print(self.x)
        

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self):
	print("measurement",self.h/(self.d-self.x))
	self.phi_hat = np.arctan2(self.h,(self.d-self.x))
	self.x = self.x + self.W*(self.phi - self.phi_hat)
	self.states += [self.x]
	print("angle ",self.phi)	
	print("phi_hat ",self.phi_hat)
	
        #return

    def run_kf(self):
	t_1 = rospy.get_time()
	self.covar()
        #current_input = self.u
        #current_measurement = self.phi
        

	#self.predict(current_input)
	if not np.isnan(self.phi):
	 
		self.measurement_update()
        
	print('state',self.x)
        self.state_pub.publish(str(float(self.x)))
	

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')    
    #rospy.init_node('get_scan_angle')
    try:
        h = 0.6 #y distance to tower
        d = 1.5 #x distance to tower (from origin)  
        
        x_0 = 0 #initial state position
        
#        Q = np.ones((1,1)) #process noise covariance        
	Q = 0.001 #process noise covariance
        R = 1 #measurement noise covariance
        P_0 = 1 #initial state covariance 
	S_0 = 1
	W_0 = 1
        kf = KalmanFilter(h, d, x_0, Q, R, P_0, S_0, W_0)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
		kf.run_kf()  
		rate.sleep()
            
    #except:
        #print(e)

    finally:
	plt.plot(kf.states)
	plt.figure()
	plt.plot(kf.covariance)
        rospy.loginfo("goodbye")

