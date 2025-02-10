#!/usr/bin/env python3
#####################################################################
# Title: Angle and rate controller
# Author: Alexnder Dowell
# Date: 12-5-21
# Purpose: PID controler for all angle rates and pitch and roll angles
#####################################################################

#-------------------------------------------------------------Imports
import os
import csv
import numpy as np
from me457common.msg import *
import rospy

v = Vehicle()
angular = Angular()
ahrs = AHRS()
rc = RC()
cmd = Servo()

#-----------------------------------------------------------Variables
#---Controller Gains---
throttle_percent = 90
yaw_percent = 60 # (deg/sec)
pitchroll_percent = 30 # (deg/sec)
rate_w_upperlimit = 1.5
rate_w_lowerlimit = 0.5
angle_w_upperlimit = 1.5
angle_w_lowerlimit = 0.5

#---RC---
# Scalers to change RC value to percent, angle, and rate values
# Scalers to change RC value to percent, angle, and rate values
perc_thr = throttle_percent / 1639
throttle_scale = perc_thr
perc_yaw = yaw_percent / 819.5
yaw_scale = perc_yaw
perc_pitchroll = pitchroll_percent / 819.5
pitchroll_scale = perc_pitchroll
# PID upper and lower limits for RC knobs
angle_uL = angle_w_upperlimit
angle_lL = angle_w_lowerlimit
angle_scale = (angle_uL - angle_lL) / 1639
angle_offset = 172 - (angle_lL / angle_scale)
rate_uL = rate_w_upperlimit
rate_lL = rate_w_lowerlimit
rate_scale = (rate_uL - rate_lL) / 1639
rate_offset = 172 - (rate_lL / rate_scale)
# Channel for throttle from RC
throttle_channel = 0
yaw_channel = 3
pitch_channel = 2
roll_channel = 1
killSW_channel = 4
rate_channel = 6
angle_channel = 5

#---Run rate (Hz)---
F_s = 125
dt = 1 / F_s

#---Butterworth Filter Setup---
f_c = 6

gamma = np.tan((np.pi * f_c) / F_s)
D = (gamma**2) + (np.sqrt(2) * gamma) + 1

#-Filter coefficients-
da_1 = 2 * ((gamma**2) - 1)
da_2 = (gamma**2) - (np.sqrt(2) * gamma) + 1

# Denominator coefficients
a_1 = da_1 / D
a_2 = da_2 / D

#-Low pass filter-
dbl_0 = gamma**2
dbl_1 = 2 * dbl_0
dbl_2 = dbl_0

# Numerator coefficients
lpf_b_0 = dbl_0 / D
lpf_b_1 = dbl_1 / D
lpf_b_2 = dbl_2 / D

#-High pass filter-
dbh_0 = 1
dbh_1 = -2
dbh_2 = 1
    
# Numerator coefficients
hpf_b_0 = dbh_0 / D
hpf_b_1 = dbh_1 / D
hpf_b_2 = dbh_2 / D

#---Complimentary Filter Coefficients---
acc_coeff = 0.98
gyro_coeff = 1 - acc_coeff

#-----------------------------------------Initialize Node & Publisher
rospy.init_node('RCcontroller')
#atti_sub = rospy.Subscriber('attipub', Angular, angle_callback)

#--------------------------------------------------------Logger Setup
i = 1
name = 'log_' + str(i) + 'controller.csv'
path = '/home/pi/catkin_ws/src/autophatros/scripts/logfiles/Alex/'

while os.path.isfile(path + name):
    i = i + 1
    name = 'log_' + str(i) + 'controller.csv'
    path = '/home/pi/catkin_ws/src/autophatros/scripts/logfiles/Alex/'

log_file = open(path + name,'w')
writer = csv.writer(log_file)

header = data = ['dt', 'throttle_SP', 'yaw_rate_SP', 'pitch_SP, roll_SP',
                'yaw_rate_act', 'yaw_rate_err_0',
                'yaw_rate_Kp', 'yaw_rate_Ki', 'yaw_rate_Kd', 'yaw_rate_compSP',
                'pitch_act', 'pitch_err_0', 
                'pitch_Kp', 'pitch_Kd', 'pitch_Ki', 'pitch_compSP',
                'pitch_rate_SP', 'pitch_rate_act', 'pitch_rate_err_0',
                'pitch_rate_Kp', 'pitch_rate_Kd', 'pitch_rate_Ki', 'pitch_rate_compSP',
                'roll_act', 'roll_err_0', 
                'roll_Kp', 'roll_Kd', 'roll_Ki', 'roll_compSP',
                'roll_rate_SP', 'roll_rate_act', 'roll_rate_err_0',
                'roll_rate_Kp', 'roll_rate_Kd', 'roll_rate_Ki', 'roll_rate_compSP']

writer.writerow(header)

#-------------------------------------------------------Def Functions
def imu_callback(message):
	global v
	v.imu = message
    
def ahrs_callback(message):
	global v
	v.ahrs = message

def rc_callback(data):
    global v
    v.rc = data

def moving_avg(x_0, x_1, x_2, x_3, x_4, x_5):
    y_n = (x_0 + x_1 + x_2 + x_3 + x_4 + x_5) / 6
    return y_n

def err_rate(y_1, y_2, dt):
    slope = (y_1 - y_2) / dt
    return slope

def trap(x_1, x_2, dt, y_1):
    area = y_1 + (((x_1 + x_2) / 2) * dt)
    return area
    
def angle(x, y):
	angle = np.degrees(np.arctan2(y, x))
	return angle

def wrap(ang):
    if ang > 180:
        dif = ang - 180
        ang = -180 + ang
    if ang < -180:
        dif = abs(ang + 180)
        ang = 180 - dif
    return ang
    
def butterworth(x_0, x_1, x_2, y_1, y_2, type):
    if type == 'LPF':
         b_0 = lpf_b_0
         b_1 = lpf_b_1
         b_2 = lpf_b_2
    elif type == 'HPF':
         b_0 = hpf_b_0
         b_1 = hpf_b_1
         b_2 = hpf_b_2
    else:
         print('No filter type')
    y_n = (b_0 * x_0) + (b_1 * x_1) + (b_2 * x_2) - (a_1 * y_1) - (a_2 * y_2)
    return y_n

def compfilt(est_1, est_2, coeff_1, coeff_2):
    est = (coeff_1 * est_1) + (coeff_2 * est_2)
    return est


ahrs_sub = rospy.Subscriber('madgwickpub', AHRS, ahrs_callback)
rc_sub = rospy.Subscriber('/rcpub', RC, rc_callback)
servocmd_pub = rospy.Publisher('servocmd', Servo, queue_size = 1)

#-------------------------------------------------------Main Function
def controller():
    rospy.on_shutdown(safe_shutdown)
    # Sends final command out to prevent leaving speed
    # commands on motors we expect to be disabled causing
    # an unsafe conditions
    
    i = 0
    j = 0

    rate = rospy.Rate(F_s)    
    zero_time = rospy.get_time()
    
    #---atti Variables---
    # Postion    
    acc_pitchP_raw_0 = 0
    acc_pitchP_raw_1 = 0
    acc_pitchP_raw_2 = 0
    acc_pitchP_raw_3 = 0
    acc_pitchP_raw_4 = 0
    acc_pitchP_raw_5 = 0
    acc_pitchP_0 = 0
    acc_pitchP_1 = 0
    acc_pitchP_2 = 0

    acc_rollP_raw_0 = 0
    acc_rollP_raw_1 = 0
    acc_rollP_raw_2 = 0
    acc_rollP_raw_3 = 0
    acc_rollP_raw_4 = 0
    acc_rollP_raw_5 = 0
    acc_rollP_0 = 0
    acc_rollP_1 = 0
    acc_rollP_2 = 0
    
    gy_p_old = 0
    gx_p_old = 0
    
    gyro_pitchP_raw_0 = 0
    gyro_pitchP_raw_1 = 0
    gyro_pitchP_raw_2 = 0
    gyro_pitchP_0 = 0
    gyro_pitchP_1 = 0
    gyro_pitchP_2 = 0
        
    gyro_rollP_raw_0 = 0
    gyro_rollP_raw_1 = 0
    gyro_rollP_raw_2 = 0
    gyro_rollP_0 = 0
    gyro_rollP_1 = 0
    gyro_rollP_2 = 0
    
    # Rate
    gyro_yawR_raw_0 = 0
    gyro_yawR_raw_1 = 0
    gyro_yawR_raw_2 = 0
    gyro_yawR_0 = 0
    gyro_yawR_1 = 0
    gyro_yawR_2 = 0
    gyro_yawR_3 = 0
    gyro_yawR_4 = 0
    gyro_yawR_5 = 0
    
    gyro_pitchR_raw_0 = 0
    gyro_pitchR_raw_1 = 0
    gyro_pitchR_raw_2 = 0
    gyro_pitchR_0 = 0
    gyro_pitchR_1 = 0
    gyro_pitchR_2 = 0
    gyro_pitchR_3 = 0
    gyro_pitchR_4 = 0
    gyro_pitchR_5 = 0
    
    gyro_rollR_raw_0 = 0
    gyro_rollR_raw_1 = 0
    gyro_rollR_raw_2 = 0
    gyro_rollR_0 = 0
    gyro_rollR_1 = 0
    gyro_rollR_2 = 0
    gyro_rollR_3 = 0
    gyro_rollR_4 = 0
    gyro_rollR_5 = 0

    #---Funcion Variables---
    # Dummy variable that will contain the single RC channel
    throttle_SP = 1
    angle_w = 1 # Weight of angle controller PID
    rate_w = 1 # Weight of rate controller PID
    
    Kp_angle = 0
    Ki_angle = 0
    Kd_angle = 0
    
    Kp_rate = 0.066610738
    Ki_rate = 0
    Kd_rate = 0.000518121
    
    Kp_yaw = 0.5
    Ki_yaw = 0.875
    Kd_yaw = 0.0075
    
    #--Angle--
    # Pitch
    pitch_SP = 0
    pitch_err_0 = 0
    pitch_err_1 = 0
    pitch_Ki = 0
    pitch_Ki = 0
    # Roll
    roll_SP = 0
    roll_err_0 = 0
    roll_err_1 = 0
    roll_Ki = 0
    roll_Ki = 0
    
    #--Rate--
    # Yaw
    yaw_rate_err_1 = 0
    yaw_rate_Ki = 0
    # Pitch
    pitch_rate_err_1 = 0
    pitch_rate_Ki = 0
    # Roll
    roll_rate_err_1 = 0
    roll_rate_Ki = 0

    
    while not rospy.is_shutdown():
        imu_sub = rospy.Subscriber('imupub', IMU, imu_callback)
        ahrs_sub = rospy.Subscriber('/madgwickpub', AHRS, ahrs_callback)

        #---Message Header---
        cmd.header.stamp = rospy.Time.now()
        now = (rospy.get_time() - zero_time)
        
        #---atti Postion---
        #-Accelerometer Estimates-
        acc_pitchP_raw_0 = v.ahrs.angular.pitch
        acc_rollP_raw_0 = v.ahrs.angular.roll
        
        # Roll
        pitch = moving_avg(acc_pitchP_raw_0, acc_pitchP_raw_1, acc_pitchP_raw_2, acc_pitchP_raw_3, acc_pitchP_raw_4, acc_pitchP_raw_5)
        roll = moving_avg(acc_rollP_raw_0, acc_rollP_raw_1, acc_rollP_raw_2, acc_rollP_raw_3, acc_rollP_raw_4, acc_rollP_raw_5)
        #-End of Accel-

        #-Gyroscope Estimates-
        gx = np.degrees(v.imu.gyroscope.x)
        gy = np.degrees(-v.imu.gyroscope.y)
        gz = np.degrees(v.imu.gyroscope.z)
        
        #-Rate-
        # Yaw
        gyro_yawR_raw_0 = gz
        gyro_yawR_0 = butterworth(gyro_yawR_raw_0, gyro_yawR_raw_1, gyro_yawR_raw_2,
                                   gyro_yawR_1, gyro_yawR_2, 'LPF')
        
        # Pitch
        gyro_pitchR_raw_0 = gy
        gyro_pitchR_0 = butterworth(gyro_pitchR_raw_0, gyro_pitchR_raw_1, gyro_pitchR_raw_2,
                                    gyro_pitchR_1, gyro_pitchR_2, 'LPF')
        
        # Roll
        gyro_rollR_raw_0 = gx
        gyro_rollR_0 = butterworth(gyro_rollR_raw_0, gyro_rollR_raw_1, gyro_rollR_raw_2,
                                   gyro_rollR_1, gyro_rollR_2, 'LPF')
		#---End of Gyro---
        
        #---Gyro Rate Filter---
        yaw_rate = moving_avg(gyro_yawR_0, gyro_yawR_1, gyro_yawR_2, gyro_yawR_3, gyro_yawR_4, gyro_yawR_5)
        pitch_rate = moving_avg(gyro_pitchR_0, gyro_pitchR_1, gyro_pitchR_2, gyro_pitchR_3, gyro_pitchR_4, gyro_pitchR_5)
        roll_rate = moving_avg(gyro_rollR_0, gyro_rollR_1, gyro_rollR_2, gyro_rollR_3, gyro_rollR_4, gyro_rollR_5)
        
        
        #---Controller---
        kill_SP = v.rc.channel[killSW_channel]
        #---RC Inputs---
        throttle_SP = (v.rc.channel[throttle_channel] - 171) * throttle_scale
        yaw_rate_SP = (v.rc.channel[yaw_channel] - 991.5) * yaw_scale
        pitch_SP = (v.rc.channel[pitch_channel] - 991.5) * -pitchroll_scale
        roll_SP = (v.rc.channel[roll_channel] - 991.5) * pitchroll_scale
        angle_w = (v.rc.channel[angle_channel] - angle_offset) * angle_scale
        rate_w = (v.rc.channel[rate_channel] - rate_offset) * rate_scale
        
        Kp_angle = 0.544234289 * angle_w
        Kd_angle = 0.001711074 * rate_w
        
        #---Angle---
        #--Pitch--
        # Data Import
        pitch_act = pitch                
        # Error
        pitch_err_0 = pitch_SP - pitch_act        
        
        #-Angle Gain Controller-
        pitch_Kp = Kp_angle * pitch_err_0
        # Derivitive Controller
        pitch_Kd = Kd_angle * err_rate(pitch_err_0, pitch_err_1, dt)
        # Intigrall Controller
        pitch_Ki = Ki_angle * trap(pitch_err_0, pitch_err_1, dt, pitch_Ki)
        # PID Controller
        pitch_compSP = pitch_Kp + pitch_Kd + pitch_Ki
        
        #--Roll--
        # Data Import
        roll_act = roll
        # Error
        roll_err_0 = roll_SP - roll_act
                
        #-Angle Gain Controller-
        roll_Kp = Kp_angle * roll_err_0
        # Derivitive Controller
        roll_Kd = Kd_angle * err_rate(roll_err_0, roll_err_1, dt)
        # Intigrall Controller
        roll_Ki = Ki_angle * trap(roll_err_0, roll_err_1, dt, roll_Ki)
        #--PID Controller--
        roll_compSP = roll_Kp + roll_Kd + roll_Ki

        
        #---Rate---
            
        #--Yaw--
        # Data Import
        yaw_rate_act = yaw_rate
        # Error
        yaw_rate_err_0 = yaw_rate_SP - yaw_rate_act
        
        #-Rate Gain Controller-
        yaw_rate_Kp = Kp_yaw * yaw_rate_err_0
        # Intigrall Controller
        yaw_rate_Ki = Ki_yaw * trap(yaw_rate_err_0, yaw_rate_err_1, dt, yaw_rate_Ki)        
        # Derivitive Controller
        yaw_rate_Kd = Kd_yaw * err_rate(yaw_rate_err_0, yaw_rate_err_1, dt)
        # PID Controller
        yaw_rate_compSP = yaw_rate_Kp + yaw_rate_Ki + yaw_rate_Kd
        
        #--Pitch--
        # Data Import
        pitch_rate_act = pitch_rate
        # Set Point
        pitch_rate_SP = pitch_compSP        
        # Error
        pitch_rate_err_0 = pitch_rate_SP - pitch_rate_act
        
        #-Rate Gain Controller-
        pitch_rate_Kp = Kp_rate * pitch_rate_err_0
        # Intigrall Controller
        pitch_rate_Ki = Ki_rate * trap(pitch_rate_err_0, pitch_rate_err_1, dt, pitch_rate_Ki)        
        # Derivitive Controller
        pitch_rate_Kd = Kd_rate * err_rate(pitch_rate_err_0, pitch_rate_err_1, dt)
        # PID Controller
        pitch_rate_compSP = pitch_rate_Kp + pitch_rate_Ki + pitch_rate_Kd
        
        #--Roll--
        roll_rate_act = roll_rate
        # Set Point
        roll_rate_SP = roll_compSP
        # Error
        roll_rate_err_0 = roll_rate_SP - roll_rate_act
        
        #-Rate Gain Controller-
        roll_rate_Kp = Kp_rate * roll_rate_err_0
        # Intigrall Controller
        roll_rate_Ki = Ki_rate * trap(roll_rate_err_0, roll_rate_err_1, dt, roll_rate_Ki)        
        # Derivitive Controller
        roll_rate_Kd = Kd_rate * err_rate(roll_rate_err_0, roll_rate_err_1, dt)
        # PID Controller
        roll_rate_compSP = roll_rate_Kp + roll_rate_Ki + roll_rate_Kd
        
        if kill_SP < 820:
            if i == 2:
                i = 0        
            #---Motor Channels---
            # Front
            cmd.channel[0] = throttle_SP - yaw_rate_compSP + pitch_rate_compSP
            # Back
            cmd.channel[1] = throttle_SP - yaw_rate_compSP - pitch_rate_compSP
            # Left
            cmd.channel[2] = throttle_SP + yaw_rate_compSP + roll_rate_compSP
            # Right
            cmd.channel[3] = throttle_SP + yaw_rate_compSP - roll_rate_compSP
            # Always write to all channels
            # Publish the message to trigger the driver callback
            servocmd_pub.publish(cmd)
            
            print ('throttle_SP: ', round(throttle_SP, 1), 'angle_w: ', round(angle_w, 3), 'rate_w: ', round(rate_w, 3))
            # print ('throttle: ', round(throttle_SP, 3), '   yaw_rate: ', round(yaw_rate_act, 3))
            # print ('pitch: ', round(pitch_act, 3),'   pitch_rate: ', round(pitch_rate_act, 3))
            # print ('roll: ', round(roll_act, 3),'     pitch_rate: ', round(roll_rate_act, 3))
            
        else:
            if j == 5:
                j = 0
                for i in range(len(cmd.channel)):
                    cmd.channel[i] = 0
                    servocmd_pub.publish(cmd)            
                print('kill_SW', 'throttle_SP: ', round(throttle_SP, 1), 'angle_w: ', round(angle_w, 3), 'rate_w: ', round(rate_w, 3))
          
        
        data = [now, throttle_SP, yaw_rate_SP, pitch_SP, roll_SP,
                yaw_rate_act, yaw_rate_err_0,
                yaw_rate_Kp, yaw_rate_Ki, yaw_rate_Kd, yaw_rate_compSP,
                pitch_act, pitch_err_0, 
                pitch_Kp, pitch_Kd, pitch_Ki, pitch_compSP,
                pitch_rate_SP, pitch_rate_act, pitch_rate_err_0,
                pitch_rate_Kp, pitch_rate_Kd, pitch_rate_Ki, pitch_rate_compSP,
                roll_act, roll_err_0, 
                roll_Kp, roll_Kd, roll_Ki, roll_compSP,
                roll_rate_SP, roll_rate_act, roll_rate_err_0,
                roll_rate_Kp, roll_rate_Kd, roll_rate_Ki, roll_rate_compSP]

        writer.writerow(data)            

        #---atti Shift---
        #-Position-
        # Accel
        acc_pitchP_raw_2 = acc_pitchP_raw_1
        acc_pitchP_raw_1 = acc_pitchP_raw_0
        acc_pitchP_2 = acc_pitchP_1
        acc_pitchP_1 = acc_pitchP_0
        acc_rollP_raw_2 = acc_rollP_raw_1
        acc_rollP_raw_1 = acc_rollP_raw_0
        acc_rollP_2 = acc_rollP_1
        acc_rollP_1 = acc_rollP_0
        # Gyro
        gy_p_old = gy
        gyro_pitchP_raw_2 = gyro_pitchP_raw_1
        gyro_pitchP_raw_1 = gyro_pitchP_raw_0
        gyro_pitchP_2 = gyro_pitchP_1
        gyro_pitchP_1 = gyro_pitchP_0
        gx_p_old = gx
        gyro_rollP_raw_2 = gyro_rollP_raw_1
        gyro_rollP_raw_1 = gyro_rollP_raw_0
        gyro_rollP_2 = gyro_rollP_1
        gyro_rollP_1 = gyro_rollP_0
        #-Rate-
        # Gyro
        gyro_yawR_raw_2 = gyro_yawR_raw_1
        gyro_yawR_raw_1 = gyro_yawR_raw_0
        gyro_yawR_5 = gyro_yawR_4    
        gyro_yawR_4 = gyro_yawR_3        
        gyro_yawR_3 = gyro_yawR_2
        gyro_yawR_2 = gyro_yawR_1
        gyro_yawR_1 = gyro_yawR_0
        gyro_pitchR_raw_2 = gyro_pitchR_raw_1
        gyro_pitchR_raw_1 = gyro_pitchR_raw_0
        gyro_pitchR_5 = gyro_pitchR_4
        gyro_pitchR_4 = gyro_pitchR_3
        gyro_pitchR_3 = gyro_pitchR_2        
        gyro_pitchR_2 = gyro_pitchR_1
        gyro_pitchR_1 = gyro_pitchR_0
        gyro_rollR_raw_2 = gyro_rollR_raw_1
        gyro_rollR_raw_1 = gyro_rollR_raw_0
        gyro_rollR_5 = gyro_rollR_4
        gyro_rollR_4 = gyro_rollR_3        
        gyro_rollR_3 = gyro_rollR_2
        gyro_rollR_2 = gyro_rollR_1
        gyro_rollR_1 = gyro_rollR_0
        
        #---Controller Shift---
        yaw_rate_err_1 = yaw_rate_err_0
        pitch_err_1 = pitch_err_0
        pitch_rate_err_1 = pitch_rate_err_0
        roll_err_1 = roll_err_0
        roll_rate_err_1 = roll_rate_err_0
        
        i = i + 1
        if i > 2:
            i = 1
        j = j + 1
        if j > 5:
            j = 1
        
        rate.sleep()
        

def safe_shutdown():
    # Upon exit set all channels to 0 for safety
    for i in range(len(cmd.channel)):
        cmd.channel[i] = 0
    servocmd_pub.publish(cmd)


if __name__ == '__main__':

    controller()