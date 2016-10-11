#!/usr/bin/env python
import rospy
from unball.msg import StrategyMessage


def talker():
    rospy.init_node('fake_strategy_node', anonymous=True)
    pub = rospy.Publisher('strategy_topic', StrategyMessage)
    r = rospy.Rate(10)
    msg = StrategyMessage()

    vel = 0.0
    signal = 0.001

    while not rospy.is_shutdown():
        #msg.lin_vel = [vel, vel, vel, vel, vel, vel]
        #msg.ang_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
        msg.lin_vel = [0.1, 0.2, 0.3, vel, vel, vel]
        msg.lin_vel = [0.3, 0.2, 0.1, vel, vel, vel]
        if vel > 0.3:
            signal = -signal
        elif vel < -0.3:
            signal = -signal
        vel += signal

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
