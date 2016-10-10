#!/usr/bin/env python

# pyserial reference: http://pyserial.readthedocs.io/en/latest/shortintro.html

import rospy
from unball.msg import StrategyMessage
from math import pi
import serial

lin_vel = [0,0,0,0,0,0]
ang_vel = [0,0,0,0,0,0]

R = 0.03
WHEELS_DISTANCE = 0.075

ser = serial.Serial()

def main():
    rospy.init_node('communication_node', anonymous=True)
    rospy.Subscriber('strategy_topic', StrategyMessage, receiveStrategyMessage, queue_size=1)

    ser.baudrate = 19200
    ser.port = '/dev/ttyACM0'
    ser.open()

    while not rospy.is_shutdown():
        rospy.spin()

def receiveStrategyMessage(data):
    rospy.logdebug('[Communication Node]Receiving Strategy message')
    for i in range(3):
        lin_vel[i] = data.lin_vel[i]
        ang_vel[i] = data.ang_vel[i]
        # Reference:
        # http://stackoverflow.com/questions/3673428/convert-int-to-ascii-and-back-in-python
        msg = str(unichr(i)) + \
              str(unichr(calculateRightSpeed(i))) + \
              str(unichr(calculateLeftSpeed(i)))
        rospy.logdebug(msg)
        if ser.isOpen():
            ser.write(msg)

def calculateLeftSpeed(i):
    linear_speed_rpm = convertSpeedToRpm(lin_vel[i])
    tangential_speed = ang_vel[i] * (WHEELS_DISTANCE / 2)
    tangential_speed_rpm = convertSpeedToRpm(tangential_speed)
    return int(linear_speed_rpm - tangential_speed_rpm)

def calculateRightSpeed(i):
    linear_speed_rpm = convertSpeedToRpm(lin_vel[i])
    tangential_speed = ang_vel[i] * (WHEELS_DISTANCE / 2)
    tangential_speed_rpm = convertSpeedToRpm(tangential_speed)
    return int(linear_speed_rpm + tangential_speed_rpm)

def convertSpeedToRpm(speed):
    wheel_length = 2*pi*R
    rotations_per_second = speed / wheel_length
    rpm = rotations_per_second * 60
    return rpm

if __name__ == '__main__':
    main()
