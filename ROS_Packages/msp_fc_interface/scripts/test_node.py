#! /usr/bin/python
import rospy
import math
from mav_msgs.msg import RateThrust
from std_msgs.msg import Bool

if __name__ == '__main__':
    rospy.init_node('msp_fc_test_node')
    pub = rospy.Publisher('/uav/control/rate_thrust', RateThrust, queue_size=1)
    pub_arm = rospy.Publisher('/uav/control/arm', Bool, queue_size=1)
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        msg = RateThrust()
        msg.angular_rates.x = 0
        msg.angular_rates.y = 0
        msg.angular_rates.z = 0
        msg.thrust.z = 0 # math.sin(i) / 2 + 0.5

        pub.publish(msg)

        arm_msg = Bool()
        arm_msg.data = True
        pub_arm.publish(arm_msg)
        i += 0.1
        rate.sleep()
