#!/usr/bin/env python

import rospy
import wiringpi2 as wp
from std_msgs.msg import Bool

AC_OK = 0
BATT_OK = 2
POWER_LATCH = 3

IN = 0
OUT = 1


def setup_gpio():
    wp.wiringPiSetup()
    wp.pinMode(AC_OK, IN)
    wp.pinMode(BATT_OK, IN)
    wp.pinMode(POWER_LATCH, OUT)
    wp.digitalWrite(POWER_LATCH, 1)


class ups_state_listener(object):

    def __init__(self):
        rospy.init_node('ups', anonymous=True)
        setup_gpio()
        self.ac_pub = rospy.Publisher('ups/ac', Bool, queue_size=1)
        self.batt_pub = rospy.Publisher('ups/battery', Bool, queue_size=1)
        rospy.Timer(rospy.Duration(1), self.update)
        rospy.spin()

    def update(self, evt):
        ac_ok = wp.digitalRead(AC_OK)
        batt_ok = wp.digitalRead(BATT_OK)
        self.ac_pub.publish(ac_ok)
        self.batt_pub.publish(batt_ok)


if __name__ == '__main__':
    ups_state_listener()
