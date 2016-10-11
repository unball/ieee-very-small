#!/usr/bin/env python

import rospy
from unball.msg import StrategyMessage
from math import pi
import serial

lin_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
ang_vel = [0.0,0.0,0.0,0.0,0.0,0.0]

R = 0.03
WHEELS_DISTANCE = 0.075

ser = serial.Serial('/dev/ttyACM1')

def main():
    rospy.init_node('communicati1on_node', anonymous=True)
    rospy.Subscriber('strategy_topic', StrategyMessage, receiveStrategyMessage, queue_size=1)

    ser.baudrate = 19200

    while not rospy.is_shutdown():
        rospy.spin()

def receiveStrategyMessage(data):
    i = 1
    #for i in range(3):
    lin_vel[i] = data.lin_vel[i]/13.
    ang_vel[i] = data.ang_vel[i]
    #rospy.logerr(lin_vel[i])
    # Reference:
    # http://stackoverflow.com/questions/3673428/convert-int-to-ascii-and-back-in-python
            
    if i == 0:
        robot_number = 2
    elif i == 1:
        robot_number = 4
    else:
        robot_number = 6

    msg = str(unichr(robot_number)) + \
          str(unichr(calculateRightSpeed(i))) + \
          str(unichr(calculateLeftSpeed(i)))
    #log = msg[0]
    #rospy.logerr(ord(log))
    #log = msg[1]
    #rospy.logerr(ord(log))
    #log = msg[2]
    #rospy.logerr(ord(log))
    #rospy.logerr("----")
    if ser.isOpen():
        ser.write(msg)

#    rospy.logdebug('[Communication Node]Receiving Strategy message')
#    for i in range(3):
#        lin_vel[i] = data.lin_vel[i]
#        ang_vel[i] = data.ang_vel[i]
#        rospy.logerror('[CommunicationNode]: ' + str((lin_vel, ang_vel)))

def calculateLeftSpeed(i):
    linear_speed_rpm = convertSpeedToRpm(lin_vel[i])
    tangential_speed = ang_vel[i] * (WHEELS_DISTANCE / 2)
    tangential_speed_rpm = convertSpeedToRpm(tangential_speed)
    speed_in_rpm = int(linear_speed_rpm - tangential_speed_rpm)
    return saturateInt(speed_in_rpm, 127)

def calculateRightSpeed(i):
    linear_speed_rpm = convertSpeedToRpm(lin_vel[i])
    tangential_speed = ang_vel[i] * (WHEELS_DISTANCE / 2)
    tangential_speed_rpm = convertSpeedToRpm(tangential_speed)
    speed_in_rpm = int(linear_speed_rpm + tangential_speed_rpm)
    return saturateInt(speed_in_rpm, 127)

def convertSpeedToRpm(speed):
    wheel_length = 2*pi*R
    rotations_per_second = speed / wheel_length
    rpm = rotations_per_second * 60
    return rpm

def saturateInt(value, max_value):
    if value > max_value:
        value = max_value
    elif value < -max_value:
        value = -max_value
    return value

if __name__ == '__main__':
    main()
