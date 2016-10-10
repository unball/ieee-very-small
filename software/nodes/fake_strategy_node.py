#!/usr/bin/env python
import rospy
from unball.msg import StrategyMessage

def talker():
    rospy.init_node('fake_strategy_node', anonymous=True)
    pub = rospy.Publisher('strategy_topic', StrategyMessage)
    r = rospy.Rate(10)
    msg = StrategyMessage()
    msg.lin_vel = [0.1,0.2,0.3,0.4,0.5,0.6]
    msg.ang_vel = [0.6,0.5,0.4,0.3,0.2,0.1]

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
