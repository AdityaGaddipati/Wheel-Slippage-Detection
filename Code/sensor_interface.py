#!/usr/bin/env python

import rospy
import math
import scipy.signal
import tf
#import tf_conversions
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion, Twist, Vector3, TransformStamped
from nmea_msgs.msg import Sentence
import numpy as np
from scipy.signal import butter, lfilter, freqz, filtfilt

rospy.init_node('Odometry_publisher')

odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "chassis"
odom.pose.covariance = [0 for i in range(36)]
#odom.pose.covariance[0] = 1
#odom.pose.covariance[7] = 1
odom.pose.pose.position.x = 0
odom.pose.pose.position.y = 0
odom.pose.pose.orientation.x = 0
odom.pose.pose.orientation.y = 0
odom.pose.pose.orientation.z = 0
odom.pose.pose.orientation.w = 1

odom.twist.covariance = [0 for i in range(36)]
odom.twist.covariance[0] = 0
odom.twist.covariance[7] = 0

quat = [0,0,0,0]
quat_vth = [0,0,0,0]

odom_trans = TransformStamped()
br = tf.TransformBroadcaster()

imu_pub = rospy.Publisher("imu_data", Imu, queue_size=50)
imu = Imu()
imu.header.frame_id = "chassis"

nmea_pub = rospy.Publisher("nmea_sentence", Sentence, queue_size=10)
nmea_msg = Sentence()
nmea_msg.header.frame_id = 'chassis'

x = 0.0
y = 0.0
th = 0.0

theta = 0
dummy_vth = 0

wheel_circumference = 0.352 #d=0.112  #0.4 #d=0.1274  #0.3947 #d = 0.1257  #0.4584    #(sprocket + track height) diameter = 0.146 #0.12 #0.136
axel_length = 0.398				
offset_r = 0
offset_p = 0
offset_y = 0

window_len = 9
buffer_len = 11
input_buffer_x = [0 for i in range(buffer_len)]
output_buffer_x = [0 for i in range(buffer_len)]
input_buffer_y = [0 for i in range(buffer_len)]
output_buffer_y = [0 for i in range(buffer_len)]
input_buffer_z = [0 for i in range(buffer_len)]
output_buffer_z = [0 for i in range(buffer_len)]
input_buffer_w = [0 for i in range(buffer_len)]
output_buffer_w = [0 for i in range(buffer_len)]
filtered_quat = [0,0,0,0]
q = [0,0,0,0]

rate = rospy.Rate(100)
rate2 = rospy.Rate(10)

current_time = rospy.get_time()
last_time = rospy.get_time()

offset_accx = 0
offset_accy = 0
offset_accz = 0
input_buffer_accX = [0 for i in range(buffer_len)]
output_buffer_accX = [0 for i in range(buffer_len)]
input_buffer_accY = [0 for i in range(buffer_len)]
output_buffer_accY = [0 for i in range(buffer_len)]
input_buffer_accZ = [0 for i in range(buffer_len)]
output_buffer_accZ = [0 for i in range(buffer_len)]

Ileft = []
Ileft_filtered = []
Iright = []
Iright_filtered = []

current_pub = rospy.Publisher("current_data", PointStamped, queue_size=10)
I = PointStamped()

order = 6
fs = 10.0       # sample rate, Hz
cutoff = 0.5  # desired cutoff frequency of the filter, Hz
b, a = butter(order, cutoff/(0.5*fs), btype='low', analog=False)

def rotate(a,q):
	ax = (1-2*(pow(q[1],2) + pow(q[2],2)))*a[0] + (2*(q[0]*q[1] - q[2]*q[3]))*a[1] + (2*(q[0]*q[2] + q[1]*q[3]))*a[2]

	ay = (2*(q[0]*q[1] + q[2]*q[3]))*a[0] + (1-2*(pow(q[0],2) + pow(q[2],2)))*a[1] + (2*(q[1]*q[2] - q[0]*q[3]))*a[2]

	az = (2*(q[0]*q[2] - q[1]*q[3]))*a[0] + (2*(q[1]*q[2] + q[0]*q[3]))*a[1] + (1-2*(pow(q[0],2) + pow(q[1],2)))*a[2]

	az = az
	print ax,ay,az
	return [ax,ay,az]

def process_data(msg):
	
	global x,y,th,quat,theta,dummy_vth,filtered_quat,q,quat_vth
	global current_time,last_time,wheel_circumference,axel_length,offset
	# global current_time_imu,last_time_imu,vx_imu,ux_imu

	if(msg.data[1] == '2'):
		rpm_data = msg.data[3:-3].split()

		if len(rpm_data) != 2:
			return

		current_time = rospy.get_time()
        	dt = current_time - last_time
		#print rpm_data,dt

		try:
			v_left = 1*( float(rpm_data[0]) )*wheel_circumference/60
			v_right = -1*( float(rpm_data[1]) )*wheel_circumference/60
		except ValueError:
			return

		vx = (v_left+v_right)/2
		vth = (v_right-v_left)/axel_length

		theta = theta + (vth*dt)
		quat_vth = tf.transformations.quaternion_from_euler(0,0,theta)
		dummy_vth = vth

		#x += vx*math.cos(theta)*dt
                #y += vx*math.sin(theta)*dt
		#br.sendTransform((x,y,0),(quat_vth),rospy.Time.now(),"chassis","odom")

		#x += vx*math.cos(th)*dt
		#y += vx*math.sin(th)*dt
		#br.sendTransform((x,y,0),(filtered_quat),rospy.Time.now(),"chassis","odom")
		#print th*(180/math.pi)
		x += vx*math.cos(th)*dt
                y += vx*math.sin(th)*dt
		#br.sendTransform((x,y,0),(quat_vth),rospy.Time.now(),"chassis","odom")

		odom.pose.pose.position.x = x
		odom.pose.pose.position.y = y
		odom.pose.pose.position.z = 0
		odom.pose.covariance[0] = 0
		odom.pose.covariance[7] = 0
		#filling the quaternion message  IMU yaw
		''' 
		odom.pose.pose.orientation.x = quat[0]
		odom.pose.pose.orientation.y = quat[1]
		odom.pose.pose.orientation.z = quat[2]
                odom.pose.pose.orientation.w = quat[3]
		'''
		'''
		# Vth yaw
		odom.pose.pose.orientation.x = quat_vth[0]
                odom.pose.pose.orientation.y = quat_vth[1]
                odom.pose.pose.orientation.z = quat_vth[2]
                odom.pose.pose.orientation.w = quat_vth[3]
		'''
		#odom.twist.twist.linear.x = vx_imu
		odom.twist.twist.linear.x = vx
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = vth

		odom.header.stamp = rospy.Time.now()
		odom_pub.publish(odom)
		
		last_time = current_time

	if(msg.data[1] == '3'):
		global offset_r,offset_p,offset_y
		imu_data1_ = msg.data[3:-3].split()

		if len(imu_data1_) != 4:
			return
		
		for val in imu_data1_:
			try:
				num = int(val)
			except ValueError:
				return

		imu_data1 = [float(i)/100 for i in imu_data1_]

		quat[0] = imu_data1[1]
		quat[1] = imu_data1[2]
		quat[2] = imu_data1[3]
		quat[3] = imu_data1[0]

		# Quat data enters fifo buffer. Median is calculated of the buffer.
		# Take the 2nd element for win_len=3 as the o/p of median filter (3rd for len=5)
		# For q.x
		input_buffer_x[1:] = input_buffer_x[:-1]
		input_buffer_x[0] = quat[0]
		output_buffer_x = scipy.signal.medfilt(input_buffer_x,window_len)		
		filtered_quat[0] = output_buffer_x[window_len/2]
		
		# For q.y
		input_buffer_y[1:] = input_buffer_y[:-1]
		input_buffer_y[0] = quat[1]
		output_buffer_y = scipy.signal.medfilt(input_buffer_y,window_len)		
		filtered_quat[1] = output_buffer_y[window_len/2]

		# For q.z
		input_buffer_z[1:] = input_buffer_z[:-1]
		input_buffer_z[0] = quat[2]
		output_buffer_z = scipy.signal.medfilt(input_buffer_z,window_len)		
		filtered_quat[2] = output_buffer_z[window_len/2]
		#print input_buffer_z
                #print output_buffer_z

		# For q.w
		input_buffer_w[1:] = input_buffer_w[:-1]
		input_buffer_w[0] = quat[3]
		output_buffer_w = scipy.signal.medfilt(input_buffer_w,window_len)		
		filtered_quat[3] = output_buffer_w[window_len/2]
		
		euler_ = tf.transformations.euler_from_quaternion(filtered_quat)

		if not (offset_r):
			offset_r = euler_[0]
		r = euler_[0]-offset_r

		if not (offset_p):
			offset_p = euler_[1]
		p = euler_[1]-offset_p

		if not (offset_y):
			offset_y = euler_[2]
		th = euler_[2]-offset_y
		
		#print r*(180/math.pi),p*(180/math.pi),y*(180/math.pi)
		filtered_quat = tf.transformations.quaternion_from_euler(r,p,th)
		q = filtered_quat
		
		odom.pose.pose.orientation.x = filtered_quat[0]
		odom.pose.pose.orientation.y = filtered_quat[1]
		odom.pose.pose.orientation.z = filtered_quat[2]
                odom.pose.pose.orientation.w = filtered_quat[3]
		
		imu.orientation.x = filtered_quat[0]
		imu.orientation.y = filtered_quat[1]
		imu.orientation.z = filtered_quat[2]
		imu.orientation.w = filtered_quat[3]
		'''
		q = tf.transformations.quaternion_from_euler(r,p,th)

		odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]

                imu.orientation.x = q[0]
                imu.orientation.y = q[1]
                imu.orientation.z = q[2]
                imu.orientation.w = q[3]
		'''
		#print euler_[0]*(180/math.pi),euler_[1]*(180/math.pi),euler_[2]*(180/math.pi)
		#br.sendTransform((x,y,0),(filtered_quat),rospy.Time.now(),"chassis","odom")

	if(msg.data[1] == '9'):
		#global offset_r,offset_p,offset_y
                mag_data1_ = msg.data[3:-3].split()

                if len(mag_data1_) != 4:
                        return

                for val in mag_data1_:
                        try:
                                num = int(val)
                        except ValueError:
                                return

                mag_data1 = [float(i)/100 for i in mag_data1_]
		mag_data1[3] *= 100

		if not (offset_y):
                        offset_y = mag_data1[3]
                th = mag_data1[3]-offset_y

		#th = mag_data1[3]

		if th>180:
			th = (th-360)
		th *= (math.pi/180)

		q = tf.transformations.quaternion_from_euler(0,0,th)
		print th*(180/math.pi)
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]


	if(msg.data[1] == '5'):

		global offset_accx,offset_accy,offset_accz
		
		imu_data2_ = msg.data[3:-3].split()

		if len(imu_data2_) != 6:
			return
		
		for val in imu_data2_:
			try:
				num = int(val)
			except ValueError:
				return

		imu_data2 = [float(i)/100 for i in imu_data2_]

		input_buffer_accX[1:] = input_buffer_accX[:-1]
		input_buffer_accX[0] = imu_data2[0]
		output_buffer_accX = scipy.signal.medfilt(input_buffer_accX,window_len)		
		accX = output_buffer_accX[window_len/2]
		
		# For q.y
		input_buffer_accY[1:] = input_buffer_accY[:-1]
		input_buffer_accY[0] = imu_data2[1]
		output_buffer_accY = scipy.signal.medfilt(input_buffer_accY,window_len)		
		accY = output_buffer_accY[window_len/2]

		# For q.z
		input_buffer_accZ[1:] = input_buffer_accZ[:-1]
		input_buffer_accZ[0] = imu_data2[2]
		output_buffer_accZ = scipy.signal.medfilt(input_buffer_accZ,window_len)		
		accZ = output_buffer_accZ[window_len/2]
		
		if not (offset_accx):
			offset_accx = imu_data2[0]
		if not (offset_accy):
			offset_accy = imu_data2[1]
		if not (offset_accz):
                        offset_accz = imu_data2[2]

		imu.linear_acceleration.x = (imu_data2[0] - offset_accx)
		imu.linear_acceleration.y = (imu_data2[1] - offset_accy)
		imu.linear_acceleration.z = imu_data2[2] - offset_accz
		'''
		ax_ = imu_data2[0]
                ay_ = imu_data2[1]
                az_ = imu_data2[2]

		accX,accY,accZ = rotate([ax_,ay_,az_],q)
		
		if not (offset_accx):
                        offset_accx = accX
                if not (offset_accy):
                        offset_accy = accY
		
		imu.linear_acceleration.x = accX - offset_accx
                imu.linear_acceleration.y = accY - offset_accy
                imu.linear_acceleration.z = accZ
		'''
		imu.angular_velocity.x = imu_data2[3]*(math.pi/180)*10
		imu.angular_velocity.y = imu_data2[4]*(math.pi/180)*10
		imu.angular_velocity.z = imu_data2[5]*(math.pi/180)*10

		imu.angular_velocity_covariance = [0.0001,0,0,0,0.0001,0,0,0,1]
		imu.linear_acceleration_covariance = [0.0090,0,0,0,0.0096,0,0,0,0.0096]
		imu.header.stamp = rospy.Time.now()
		imu_pub.publish(imu)

		#odom.header.stamp = rospy.Time.now()
		#odom_pub.publish(odom)

	if(msg.data[1] == '7'):
		global Ileft,Ileft_filtered,Iright,Iright_filtered
		global a,b

		data_log = msg.data[3:-3].split()
		#print float(data_log[6]),float(data_log[11])

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

		I.point.x = Ileft_filtered[-1]
		I.point.y = Iright_filtered[-1]
		I.header.stamp = rospy.Time.now()
		current_pub.publish(I)
		
	if(msg.data[0] == '$'):
		nmea_msg.sentence = msg.data[:-2]
		nmea_msg.header.stamp = rospy.Time.now()
		nmea_pub.publish(nmea_msg)

def process_zed_imu(imu_msg):
	global offset_y,th,quat,x,y

	quat[0] = imu_msg.orientation.x
	quat[1] = imu_msg.orientation.y
	quat[2] = imu_msg.orientation.z
	quat[3] = imu_msg.orientation.w

	euler_ = tf.transformations.euler_from_quaternion(quat)

	if not (offset_y):
		offset_y = euler_[2]
	th = euler_[2]-offset_y

	quat_vth = tf.transformations.quaternion_from_euler(0,0,th)

	odom.pose.pose.orientation.x = quat_vth[0]
	odom.pose.pose.orientation.y = quat_vth[1]
	odom.pose.pose.orientation.z = quat_vth[2]
	odom.pose.pose.orientation.w = quat_vth[3]

	#br.sendTransform((x,y,0),(quat_vth),rospy.Time.now(),"chassis","odom")

data_sub = rospy.Subscriber('/due_data', String, process_data, queue_size=100)
#zed_imu_sub = rospy.Subscriber('/imu/data', Imu, process_zed_imu, queue_size=100)

while not rospy.is_shutdown():
	rate.sleep()
