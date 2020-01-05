#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz, filtfilt
import matplotlib.pyplot as plt
import time

rospy.init_node('Slip_Detection')

Va = 0
Vr = 0
wheel_circumference = 0.352

Ileft = []
Ileft_filtered = []
Iright = []
Iright_filtered = []
Iavg = 0

per = []
per_filtered = []

pub = rospy.Publisher("velocity", PointStamped, queue_size=10)
v = PointStamped()

order = 6
fs = 10.0       # sample rate, Hz
cutoff = 0.5  # desired cutoff frequency of the filter, Hz
b, a = butter(order, cutoff/(0.5*fs), btype='low', analog=False)

x = []
y = []
x1 = np.array([])

count = [0,0,0,0,0,0,0,0,0,0]

Il = 0
Ir = 0
n = 0

Islip = 0.092

def process_data(msg):

	if(msg.data[1] == '7'):
		global Ileft,Ileft_filtered,Iright,Iright_filtered,Il,Ir,n
		global a,b,Va,Vr,wheel_circumference
		global last,x,y, Islip

		data_log = msg.data[3:-3].split()

		if len(data_log) != 13:
			return

		try:
			num1 = float(data_log[6])
			num2 = float(data_log[11])
			num3 = float(data_log[4])
			num4 = float(data_log[9])
		except ValueError:
			return

		if len(Ileft) < 10:
			Ileft.append(float(data_log[6]))
			Iright.append(float(data_log[11]))
			return

		Ileft.pop(0)
		Ileft.append(float(data_log[6]))
		Ileft_filtered = filtfilt(b, a, Ileft, padlen=9)

		Iright.pop(0)
		Iright.append(float(data_log[11]))
		Iright_filtered = filtfilt(b, a, Iright, padlen=9)

		'''
		I.point.x = Ileft_filtered[-1]
		I.point.y = Iright_filtered[-1]
		I.point.z = Iright[-1]
		I.header.stamp = rospy.Time.now()
		current_pub.publish(I)
		'''
		Iavg = (Ileft_filtered[-1] + Iright_filtered[-1])/2
		#Iavg = Iright_filtered[-1]

		'''
		n += 1
		Il = Il + Ileft_filtered[-1]
		Ir = Ir + Iright_filtered[-1]
		Ilavg = Il/n
		Iravg = Ir/n
		print Ilavg,Iravg
		'''

		v_left = ( float(data_log[4]) )*wheel_circumference/60
		v_right = -1*( float(data_log[9]) )*wheel_circumference/60
		Vr = (v_left + v_right)/2
		#Vr = v_right	

		#print Vr,Va

		x.append(Iavg)
		y.append(Vr-Va)		

		last = time.time()

		#calc_slip()

		if Iavg>Islip:
			slip_correction(Iavg)
	
def calc_slip():
		global Va,Vr,count
		global per,per_filtered,a,b
		try:
			p = (1 - Va/Vr)*100
			if p>=0 and p<=100:
				
				if len(per) < 10:
					per.append(p)
					return

				per.pop(0)
				per.append(p)
				per_filtered = filtfilt(b, a, per, padlen=9)
				#print per_filtered[-1]

				count[int(per_filtered[-1]/10)] += 1
				#print count

		except ZeroDivisionError:
			return

def slip_correction(Iavg):
	global Va,Vr,Islip

	m = 0.07072152
	c = -0.00650167

	Sc = m*(Iavg-Islip)

	#print Sc,Iavg

	v.point.x = Va
	v.point.y = Vr
	v.point.z = Vr - Sc
	v.header.stamp = rospy.Time.now()
	pub.publish(v)

def odom_cb(msg1):
		global Va
		Va = msg1.twist.twist.linear.x

data_sub = rospy.Subscriber('/due_data', String, process_data, queue_size=100)

odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_cb, queue_size=100)

now = time.time()
last = now

rate = rospy.Rate(100)
while not rospy.is_shutdown():
	if now-last>3 and last:
		p = np.polyfit(x,y,1)
		print p
		x1 = np.append(x1,x)
		plt.plot(x, y, 'o', label='S vs I', markersize=10)
		plt.plot(x1, p[0]*x1 + p[1], 'r', label='Fitted line')
		plt.legend()		
		plt.grid()
		plt.show()
		last = 0
	rate.sleep()
	now = time.time()
