#!/usr/bin/env python
import rospy
from unball.msg import StrategyMessage

def talker():
    rospy.init_node('fake_strategy_node', anonymous=True)
    pub = rospy.Publisher('strategy_topic', StrategyMessage)
    r = rospy.Rate(10)
    msg = StrategyMessage()
    msg.lin_vel = [1,2,3,4,5,6]
    msg.ang_vel = [6,5,4,3,2,1]

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
