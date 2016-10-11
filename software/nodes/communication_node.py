#!/usr/bin/env python

import rospy
from unball.msg import StrategyMessage
from math import pi

lin_vel = [0,0,0]
ang_vel = [0,0,0]

R = 0.03
WHEELS_DISTANCE = 0.075


def main():
    rospy.init_node('communication_node', anonymous=True)
    rospy.Subscriber('strategy_topic', StrategyMessage, receiveStrategyMessage, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()

def receiveStrategyMessage(data):
    rospy.logdebug('[Communication Node]Receiving Strategy message')
    for i in range(3):
        lin_vel[i] = data.lin_vel[i]
        ang_vel[i] = data.ang_vel[i]
        rospy.loginfo('[CommunicationNode]: ' + str((lin_vel, ang_vel)))

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
