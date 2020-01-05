#!/usr/bin/env python

import serial
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robot_localization.srv import PumpTrig

#kp = ' 010'

heartbeat_packet = '<4>'
beat_time = 0
due_connected = False

pump_packet_read = '<6>'
pump_done = False

linear_x = 0.0
angular_z = 0.0

min_velX = -0.2
max_velX = 0.2
min_angZ = -0.5
max_angZ = 0.5

wheel_circumference = 0.352 #d=0.112  #0.4 #d=0.1274  #0.314 #d=0.1  #0.3947 #d = 0.1257   #(sprocket + track height) diameter = 0.146
axel_length = 0.398

#ser = serial.Serial('/dev/ttyTHS2', 115200, timeout=1)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

pub = rospy.Publisher('/due_data', String, queue_size=1000)

def cmd_vel_callback(data):
    #print data
    global linear_x, angular_z
    if data.linear.x == linear_x and data.angular.z == angular_z:
        return
    else:
        linear_x = data.linear.x
        angular_z = data.angular.z

        send_desired_rpm(linear_x, angular_z)

def read_ser():
    global beat_time, due_connected, heartbeat_packet,pump_packet_read,pump_done
    if ser.in_waiting > 0:
        read_str = ser.readline()
        if heartbeat_packet in read_str:
        # print "beat received"
            beat_time = time.clock()
	    due_connected = True
	    ser.write(heartbeat_packet.encode())
        elif pump_packet_read in read_str:
            pump_done = True
        else:
            pub.publish(read_str)
    if time.clock() - beat_time > 1:
        due_connected = False

def PumpNow(req):
    global pump_done, angle, velocity
    pump_packet = '<6 ' + zero_pad_to_str(req.angle) + ' ' + zero_pad_to_str(req.velocity) + '>'
    ser.write(pump_packet.encode())
    while not pump_done:
        read_ser()
        time.sleep(0.1)
    pump_done = False
    return True, "Pump Done"

def arduino_interface():
    rospy.init_node('arduino_interface', anonymous=True)

    s = rospy.Service('PumpService', PumpTrig, PumpNow)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    while not rospy.is_shutdown():
        # do whatever you want here
        # print due_connected
        read_ser()

def constrain(x, min_val, max_val):
    if x > max_val:
        return max_val
    elif x < min_val:
        return min_val
    else:
        return x

def zero_pad_to_str(val):
    val = int(abs(val))
    if val < 10:
        return '00' + str(val)
    elif val < 100:
        return '0' + str(val)
    elif val < 1000:
        return str(val)
    else:
        return '000'

def send_desired_rpm(linear_x, angular_z):
    # cmd_string = <X 000 000 0 0> : <cmd_id left_rpm right_rpm left_dir right_dir>
    # print "sending!"

    global min_velX,max_velX,min_angZ,max_angZ,kp
    #print "-------------"
    #print linear_x, angular_z
    #linear_x = constrain(linear_x,min_velX,max_velX)
    #angular_z = constrain(angular_z,min_angZ,max_angZ)
    #print linear_x, angular_z
    v_left = linear_x - (angular_z*axel_length)/2
    v_right = linear_x + (angular_z*axel_length)/2

    left_desired_rpm = round(v_left*(60/wheel_circumference))
    right_desired_rpm = round(v_right*(60/wheel_circumference))

    left_dir = '0' if (left_desired_rpm > 0) else '1'
    right_dir = '1' if (right_desired_rpm > 0) else '0'

    cmd_string = '<1 ' + zero_pad_to_str(left_desired_rpm) \
                 + ' ' + zero_pad_to_str(right_desired_rpm) \
                 + ' ' + left_dir + ' ' + right_dir + '>'

    #cmd_string = '<1 003 015 1 1 010>'
    ser.write(cmd_string.encode())
    print cmd_string

if __name__ == '__main__':
    try:
        arduino_interface()
    except rospy.ROSInterruptException:
        pass
