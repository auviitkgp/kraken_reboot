#!/usr/bin/env python


import serial
import math
import struct
import numpy as np
import rospy
import sys
import os
import tf

# No need to use the below import, use the import in line 18 instead.
#from kraken_msgs.msg import imuData_new

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Vector3 , Quaternion
from resources import topicHeader as th
imu_pub = rospy.Publisher(th.SENSOR_IMU, Imu, queue_size = 2)
#mag_pub = rospy.Publisher(th.SENSOR_IMU_MAGNETOMETER, MagneticField, queue_size = 2)
#temp_pub = rospy.Publisher(th.SENSOR_IMU_TEMPERATURE, Temperature, queue_size =2)

rospy.init_node('IMU_Data_Publisher', anonymous=True)
## Code to find port automatically
find = os.popen('dmesg | grep FTDI')
port = find.read()
num = port.split('now attached to ')
portName = '/dev/'+(num[1].split('\n'))[0]
##
'''
if (len(sys.argv) == 2):
    num = str(sys.argv[1])
else:
    num = '0'
'''
imu = serial.Serial(portName, 115200)


# DVL config
imu.stopbits = 2
#

# Variables
roll = 0.0
pitch = 0.0
yaw = 0.0
ax = 0.0
ay = 0.0
az = 0.0
mx = 0.0
my = 0.0
mz = 0.0
gx = 0.0
gy = 0.0
gz = 0.0
temp = 0.0
magError = 0.0
g = 0.0
quat_ang = 0.0
quat_vect = [0.0] * 3
##


def setBaud(rate):
    if (rate == 300):
        message = '$PSPA,BAUD=0\r\n'
    elif (rate == 1200) :
        message = '$PSPA,BAUD=1\r\n'
    elif (rate == 2400) :
        message = '$PSPA,BAUD=2\r\n'
    elif (rate == 4800) :
        message = '$PSPA,BAUD=3\r\n'
    elif (rate == 9600) :
        message = '$PSPA,BAUD=4\r\n'
    elif (rate == 19200) :
        message = '$PSPA,BAUD=5\r\n'
    elif (rate == 38400) :
        message = '$PSPA,BAUD=6\r\n'
    elif (rate == 57600) :
        message = '$PSPA,BAUD=7\r\n'
    elif (rate == 115200) :
        message = '$PSPA,BAUD=8\r\n'

    imu.write(message)
    garbage = ''
    total = ''
    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

def temperature():
    command = '$PSPA,TEMP\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []
    global temp
    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    temp = float((data[1]).split('='))

def quat():
    command = '$PSPA,QUAT\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []
    global quat
    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    quat_ang = float(((data[1]).split('='))[1])
    quat_vect[0] = float(((data[2]).split('='))[1])
    quat_vect[1] = float(((data[3]).split('='))[1])
    quat_vect[2] = float(((data[4].split('*'))[0].split('='))[1])

def rpyt():
    command = '$xxXDR,\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []

    global roll
    global pitch
    global yaw
    global temp
    global magError

    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')

    yaw = float(data[5])
    pitch = float(data[8])
    roll = float(data[11])
    temp = float(data[14])
    magError = float(((data[17]).split('*'))[0])

def accelero():
    command = '$PSPA,A\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    global ax
    global ay
    global az
    global g

    data = []

    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    #print data
    ax = -float(((data[1]).split('='))[1])*g/1000.0
    ay = -float(((data[2]).split('='))[1])*g/1000.0
    az = -float(((((data[3]).split('='))[1]).split('*'))[0])*g/1000.0

def gyro():
    command = '$PSPA,G\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []
    global gx
    global gy
    global gz

    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    gx = float(((data[1]).split('='))[1])*np.pi/180000.0
    gy = float(((data[2]).split('='))[1])*np.pi/180000.0
    gz = float(((((data[3]).split('='))[1]).split('*'))[0])*np.pi/180000.0

def magneto():
    command = '$PSPA,M\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []
    global mx
    global my
    global mz

    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    mx = float(((data[1]).split('='))[1])
    my = float(((data[2]).split('='))[1])
    mz = float(((((data[3]).split('='))[1]).split('*'))[0])

def pitchRoll():
    command = '$PSPA,PR\r\n'
    imu.write(command)
    garbage = ''
    total = ''
    data = []
    global pitch
    global roll

    while (garbage != '\r'):
        garbage = imu.read()
        total += garbage
    #print total

    data = total.split(',')
    ax = float(((data[1]).split('='))[1])
    ay = float(((data[2]).split('='))[1])
    az = float(((((data[3]).split('='))[1]).split('*'))[0])

def getOrientationCovariance():

    # global roll
    # global pitch
    # global yaw

    if rospy.has_param('OrientationCov_mat'):
        cov_mat = map(float,rospy.get_param('OrientationCov_mat')[1:-1].split(','))
    else:
        cov_mat = [0.0] * 9
    return cov_mat

def getAngularVelocityCovariance():

    # global gx
    # global gy
    # global gz
    if rospy.has_param('AngularVelCov_mat'):
        cov_mat = map(float,rospy.get_param('AngularVelCov_mat')[1:-1].split(','))
    else:
        cov_mat = [0.0] * 9
    return cov_mat

def getLinearAccelerationCovariance():

    # global ax
    # global ay
    # global az
    if rospy.has_param('LinearAccelerationCov_mat'):
        cov_mat = map(float,rospy.get_param('LinearAccelerationCov_mat')[1:-1].split(','))
    else:
        cov_mat = [0.0] * 9
    return cov_mat

def new_msg_format():

    global roll
    global pitch
    global yaw
    global ax
    global ay
    global az
    global mx
    global my
    global mz
    global gx
    global gy
    global gz
    global temp
    global magError

    accelero()
    gyro()
    magneto()
    rpyt()

    #msg1 = imuData_new()

    imu_msg = Imu()
    mag_msg = MagneticField()
    temp_msg = Temperature()
    # print roll, pitch, yaw
    # Fix the roll, pitch by subtracting it from 360

    roll  = 360 - roll
    pitch = 360 - pitch

    roll = roll % 360
    pitch = pitch % 360
    yaw = yaw % 360

    quaternion_ned = tf.transformations.quaternion_from_euler(roll*np.pi/180.0, pitch*np.pi/180.0, yaw*np.pi/180.0)
    enu_to_ned= tf.transformations.quaternion_from_euler(np.pi,0,np.pi/2)
    quaternion_enu=tf.transformations.quaternion_multiply(enu_to_ned,quaternion_ned)

    #rpy_enu=tf.transformations.euler_from_quaternion(quaternion_enu)
    #print "\nRoll_NED= "+str(roll)+"\tPitch_NED= "+str(pitch)+"\tYaw_NED= "+str(yaw)
    #print "Roll_ENU= "+str(rpy_enu[0]*180.0/np.pi)+"\tPitch_ENU= "+str(rpy_enu[1]*180.0/np.pi)+"\tYaw_ENU= "+str(rpy_enu[2]*180.0/np.pi)
    #print "Acceleration Magnitude= "+str(np.sqrt(ax**2+ay**2+az**2))

    ## Offset Calculation (Done Offline)
    # g_vector=np.array([0,0,g,1.0])
    # RotMat_enu=tf.transformations.quaternion_matrix(quaternion_enu)
    # g_body=np.dot(RotMat_enu.transpose(),g_vector)[0:3]
    # a_vector=np.array([ax,ay,az])
    # offset=a_vector-g_body
    # offset_world=np.dot(RotMat_enu,np.append(offset,1.0))[0:3]
    # print "Offset Vector= "+str(offset_world)+"\nOffset Magnitude= "+str(np.linalg.norm(offset_world))

    imu_msg.header.stamp=rospy.Time.now()
    mag_msg.header.stamp=rospy.Time.now()
    temp_msg.header.stamp=rospy.Time.now()
    imu_msg.header.frame_id='imu'
    mag_msg.header.frame_id='imu'
    temp_msg.header.frame_id='imu'
    imu_msg.orientation = Quaternion(quaternion_enu[0],quaternion_enu[1],quaternion_enu[2],quaternion_enu[3])
    imu_msg.orientation_covariance = getOrientationCovariance()
    imu_msg.angular_velocity = Vector3(gx,gy,gz)
    imu_msg.angular_velocity_covariance = getAngularVelocityCovariance()
    imu_msg.linear_acceleration = Vector3(ax,ay,az)
    imu_msg.linear_acceleration_covariance = getLinearAccelerationCovariance()
    mag_msg.magnetic_field = Vector3(mx,my,mz)
    temp_msg.temperature = temp

    return imu_msg , mag_msg , temp_msg

if __name__ == '__main__':

    #global g
    pubData = [0.0] * 13
    g=rospy.get_param('gravitational_acceleration',9.80665)
    if (not imu.isOpen) :
        imu.close()
        imu.open()

    if (imu.isOpen) :
        print 'Serial port opened successfully'
    else:
        print 'Error in opening port'

    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        imu_msg , mag_msg , temp_msg = new_msg_format()
        imu_pub.publish(imu_msg)
        #mag_pub.publish(mag_msg)
        #temp_pub.publish(temp_msg)
        r.sleep()

    imu.close()
